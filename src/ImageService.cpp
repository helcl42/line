#include "ImageService.h"
#include "DetectionParams.h"
#include "Line.h"

ImageService::ImageService(DetectionSettings* settings)
: m_shrink(2), m_settings(settings), m_settingsIndex(0)
{
    m_image = new Image<float>();
    m_colorImage = new Image<float>();
    m_lineDetector = new LineDetector(settings->getItem(0));
}

ImageService::~ImageService()
{
    SAFE_DELETE(m_image);
    SAFE_DELETE(m_colorImage);
    SAFE_DELETE(m_lineDetector);
}

Vector2<int>* ImageService::perform(const sensor_msgs::Image::ConstPtr& img)
{
    unsigned long timeElapsed;
    Vector2<int>* objectPoint = NULL;

    m_shrinkTimer.start();

    m_image->setInstance(img, m_shrink);
    m_colorImage->setInstance(img, m_shrink);

    m_lineDetector->invalidate();
    m_lineDetector->setImages(m_image, m_colorImage);
    LinePair* line = m_lineDetector->detectLine();

    if (line->isValid())
    {
        std::cout << "CARA!!! " << line->getFirst()->length << std::endl;

        if (m_changeColorTimer.isStarted())
        {
            m_changeColorTimer.stop();
        }
        writeLinesToMessage(img, line->getLines(), 2);
        
        objectPoint = getObjectPoint(line);
        if(objectPoint != NULL)
        {
            writePointToMessage(img, objectPoint);
        }
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

            m_lineDetector->setColorSettings(m_settings->getItem(m_settingsIndex));
        }
    }

    writeImageToMessage(img);

    m_shrinkTimer.stop();

    timeElapsed = m_shrinkTimer.getElapsedTimeInMicroSec();
    std::cout << "Elapsed " << timeElapsed << "ms " << m_shrinkTimer.getFPS() << " FPS" << std::endl;

    if (timeElapsed > 450000)
    {
        if (m_shrink < 6) m_shrink++;
    }
    else if (timeElapsed < 150000)
    {
        if (m_shrink > 1) m_shrink--;
    }

    DetectionParams::recomputeMetrics(img->width, img->height, m_shrink);
    std::cout << "Params: len = " << DetectionParams::lineLengthTreshold << std::endl;

    return objectPoint;
}

Vector2<int>* ImageService::getObjectPoint(LinePair* line)
{
    Line* l1 = line->getFirst();
    Line* l2 = line->getSecond();
    unsigned int halfLength;

    if (l1->points.size() < l2->points.size())
    {
        halfLength = l1->points.size() / 2;
    }
    else
    {
        halfLength = l2->points.size() / 2;
    }

    return Vector2<int>::getPointBetween(l1->points[halfLength], l2->points[halfLength]);
}

void ImageService::writePointToMessage(const sensor_msgs::Image::ConstPtr& img, Vector2<int>* point, unsigned int size)
{

    unsigned char* temp;
    unsigned long imageSize = img->height * img->width * 3;

//    if(size % 2 == 0) size++;    
    //TODO check bounds
    
    unsigned int minX;
    unsigned int maxX;
    unsigned int minY;
    unsigned int maxY;
    
    if(size == 1) 
    {
        minX = minY = 1;        
        maxX = maxY = 1;        
    }
    else 
    {
        minX = point->x - size / 2;
        maxX = point->x + size / 2;
        minY = point->y - size / 2;
        maxY = point->y + size / 2;
    }

    for (unsigned int i = minX; i < maxX; i++)
    {
        for (unsigned int j = minY, index = 0; j < maxY; j++)
        {
            index = imageSize - (j + 1) * img->width * 3 * m_shrink + (m_image->getWidth() - i) * m_shrink * 3;
            
            temp = (unsigned char*) &img->data[index];
            *temp = (unsigned char) 255;
            temp = (unsigned char*) &img->data[index + 1];
            *temp = (unsigned char) 255;
            temp = (unsigned char*) &img->data[index + 2];
            *temp = (unsigned char) 255;
        }
    }
}

void ImageService::writeLinesToMessage(const sensor_msgs::Image::ConstPtr& img, Line** line, unsigned int count, unsigned int width)
{
    Line* oneLine = NULL;        
    
    if (img->width > 0 && img->height > 0)
    {                
        for (unsigned int i = 0; i < count; i++)
        {
            oneLine = line[i];
            for (unsigned int j = 0; j < oneLine->points.size(); j++)
            {
//                index = size - (oneLine->points[j].y + 1) * img->width * 3 * m_shrink + (m_image->getWidth() - oneLine->points[j].x) * m_shrink * 3;
//
//                temp = (unsigned char*) &img->data[index];
//                *temp = (unsigned char) 0;
//                temp = (unsigned char*) &img->data[index + 1];
//                *temp = (unsigned char) 0;
//                temp = (unsigned char*) &img->data[index + 2];
//                *temp = (unsigned char) 255;                
                writePointToMessage(img, &(oneLine->points[j]), width);
            }
        }
    }
}

void ImageService::writeImageToMessage(const sensor_msgs::Image::ConstPtr& img)
{
    if (img->width > 0 && img->height > 0)
    {
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

            index = size - (i + 1) * img->width * 3 + img->width * 3;
        }
    }
}

unsigned int ImageService::getShrink() const
{
    return m_shrink;
}
