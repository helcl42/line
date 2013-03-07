#include "ImageService.h"
#include "DetectionParams.h"
#include "Line.h"

int counter = 0;

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

void ImageService::perform(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::Image::ConstPtr& depth)
{
    unsigned long timeElapsed;

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
        getWayPoint(line, depth);
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
        if(m_shrink < 6) m_shrink++;
    }
    else if (timeElapsed < 150000)
    {
        if (m_shrink > 1) m_shrink--;        
    }

    DetectionParams::recomputeMetrics(img->width, img->height, m_shrink);

    std::cout << "Params: len = " << DetectionParams::lineLengthTreshold << std::endl;
    
//    
//    counter++;
//    if(counter > 3) {
//        ros::shutdown();
//    }
}

Vector2<float>* ImageService::getWayPoint(LinePair* line, const sensor_msgs::Image::ConstPtr& depth)
{
    Line* l1 = line->getFirst();
    Line* l2 = line->getSecond();
    
    float distance;
    unsigned int index;
    unsigned int thirdLength = l1->points.size() < l2->points.size() ? l1->points.size() / 3: l2->points.size() / 3;

    Vector2<int> midPoint = Vector2<int>::getPointBetween(l1->points[thirdLength], l2->points[thirdLength]);      
    
    std::cout << "MID point: " << midPoint << std::endl;
    
    index = midPoint.x * 4 * m_shrink + midPoint.y * depth->width * 4 * m_shrink;
    BYTES_TO_FLOAT_L(distance, depth->data, index);
    
    std::cout << "distance = " << distance << std::endl; 
    
    return NULL;
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
                index = size - (oneLine->points[j].y + 1) * img->width * 3 * m_shrink + (m_image->getWidth() - oneLine->points[j].x) * m_shrink * 3;

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
