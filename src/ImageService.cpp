#include "ImageService.h"
#include "DetectionParams.h"
#include "Line.h"

ImageService::ImageService(DetectionSettings* settings)
: m_shrink(1), m_settings(settings), m_settingsIndex(0)
{
    m_image = new BmpImage<float>();
    m_colorImage = new BmpImage<float>();
    m_strategy = new SobelStrategy(settings->getItem(0));
}

ImageService::~ImageService()
{
    SAFE_DELETE(m_image);
    SAFE_DELETE(m_colorImage);
    SAFE_DELETE(m_strategy);
}

void ImageService::perform(const sensor_msgs::Image::ConstPtr& img)
{
    unsigned long timeElapsed;

    m_shrinkTimer.start();
    m_image->setInstance(img, m_shrink);
    m_colorImage->setInstance(img, m_shrink);

    m_strategy->setImages(m_image, m_colorImage);

    LinePair* line = m_strategy->detectLine();

    if (line->isValid())
    {
        std::cout << "CARA!!! " << line->getFirst()->length << std::endl;

        if (m_changeColorTimer.isStarted())
        {
            m_changeColorTimer.stop();
        }
        writeLinesToMessage(img, line->getLines(), 2); 
    }
    else
    {
        if (!m_changeColorTimer.isStarted())
        {
            m_changeColorTimer.start();
        }
        else if (m_changeColorTimer.getElapsedTimeInMilliSec() > 5000) //pokud po 5 vterinach neuvidi hledanou caru, hleda dalsi
        {
            m_changeColorTimer.stop();
            std::cout << "Hledam dalsi!!" << std::endl;

            m_settingsIndex++;
            if (m_settingsIndex >= m_settings->getCountOfColors())
            {
                //detect home shapes
                //m_strategy->detectRectangle();
                m_settingsIndex = 0;
            }

            m_strategy->setSettings(m_settings->getItem(m_settingsIndex));
        }
    }

//    Line * lines[2];
//    Line l;
//    for (int i = 0, j = 0; i < m_image->getHeight(); i++, j++)
//    {
//        l.points.push_back(Vector2<int>(i, j));
//    }
//
//    Line lineReverse;
//    for (int i = m_image->getHeight() - 1, j = 0; i >= 0; i--, j++)
//    {
//        lineReverse.points.push_back(Vector2<int>(i, j));
//    }
//
//    lines[0] = &l;
//    lines[1] = &lineReverse;
//
//    writeLinesToMessage(img, lines, 2);
    
    writeImageToMessage(img);

    m_shrinkTimer.stop();   
    
    timeElapsed = m_shrinkTimer.getElapsedTimeInMicroSec();
    std::cout << "Elapsed " << timeElapsed << "ms " << m_shrinkTimer.getFPS() << " FPS" << std::endl;
    if (timeElapsed > 450000)
    {
        m_shrink++;
    }
    else if (timeElapsed < 150000)
    {
        if (m_shrink < 5 && m_shrink > 1)
        {
            m_shrink--;
        }
    }    
    
    DetectionParams::recomputeMetrics(img->width, img->height, m_shrink);

    std::cout << "Params: len = " << DetectionParams::lineLengthTreshold << std::endl;
}

void ImageService::writeLinesToMessage(const sensor_msgs::Image::ConstPtr& img, Line** line, unsigned int count, unsigned int width)
{
    if (img->width > 0 && img->height > 0)
    {        
        unsigned char* temp;
        Line* oneLine = NULL;
        unsigned long size = img->height * img->width * 3;       
        
        for (unsigned int i = 0; i < count; i++)
        {
            oneLine = line[i];
            for (unsigned int j = 0, index = 0; j < oneLine->points.size(); j++)
            {
                index = size - (oneLine->points[j].y + 1) * img->width * 3 * m_shrink + (m_image->getWidth() - oneLine->points[j].x)  * m_shrink * 3;

                temp = (unsigned char*) &img->data[index];
                *temp = (unsigned char) 0;
                temp = (unsigned char*) &img->data[index + 1];
                *temp = (unsigned char) 0;
                temp = (unsigned char*) &img->data[index + 2];
                *temp = (unsigned char) 255;
            }
        }
    }
}

void ImageService::writeImageToMessage(const sensor_msgs::Image::ConstPtr& img)
{
    if (img->width > 0 && img->height > 0)
    {
        //        short* widthPtr = (short*)img->width;
        //        *widthPtr = m_width;
        //        short* heightPtr = (short*)img->height;
        //        *heightPtr = m_height;

        Pixel<float>* pixel = NULL;
        unsigned char* temp;
        unsigned long size = img->height * img->width * 3;

        for (unsigned int i = 0, index = img->width * 3; i < m_image->getHeight(); i++)
        {
            for (unsigned int j = 0; j < m_image->getWidth(); j++, index -= 3)
            {
                pixel = m_image->getPixel(i, j);
                temp = (unsigned char*) &img->data[index + 2];
                *temp = (unsigned char) pixel->b;
                temp = (unsigned char*) &img->data[index + 1];
                *temp = (unsigned char) pixel->g;
                temp = (unsigned char*) &img->data[index];
                *temp = (unsigned char) pixel->r;
            }
            //index = (i + 1) * img->width * 3;                
            index = size - (i + 1) * img->width * 3 + img->width * 3;
        }
    }
}
