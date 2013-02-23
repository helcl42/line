#include "ImageService.h"
#include "DetectionParams.h"

ImageService::ImageService(DetectionSettings* settings)
: m_shrink(1), m_settings(settings)
{
    m_image = new BmpImage<float>();
    m_colorImage = new BmpImage<float>();
    m_strategy = new SobelStrategy(settings);
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

    m_shrink = 2;
    m_timer.start();
    m_image->setInstance(img, m_shrink);
    m_colorImage->setInstance(img, m_shrink);

    m_strategy->setImages(m_image, m_colorImage);
    BestLine* line = m_strategy->detectLine();

    if (line->isValid())
    {
        std::cout << "CARA!!! " << line->getFirst()->length << std::endl;
    }

    writeImageToMessage(img);

    m_timer.stop();

    timeElapsed = m_timer.getElapsedTimeInMicroSec();
    std::cout << "Elapsed " << timeElapsed << std::endl;
    if (timeElapsed > 300000)
    {
        m_shrink++;
    }
    else if (timeElapsed < 100000)
    {
        if (m_shrink < 5 && m_shrink > 1)
        {
            m_shrink--;
        }
    }
    //DetectionParams::recomputeMatrics(m_shrink);

    std::cout << "Line len = " << DetectionParams::lineLengthTreshold << std::endl;
}

void ImageService::writeLineToMessage(const sensor_msgs::Image::ConstPtr& img, Line** line, unsigned int width)
{
    if (img->width > 0 && img->height > 0)
    {
        unsigned char* temp;
        Line* oneLine = NULL;

        for (unsigned int i = 0; i < 2; i++)
        {
            oneLine = line[i];
            for (unsigned int j = 0, index = 0; j < oneLine->points.size(); j++)
            {
                index = i * img->width * 3;

                for (unsigned int k = index - width / 2; k < index + width / 2; k++)
                {
                    temp = (unsigned char*) &img->data[k];
                    *temp = (unsigned char) 255;
                    temp = (unsigned char*) &img->data[k + 1];
                    *temp = (unsigned char) 0;
                    temp = (unsigned char*) &img->data[k + 2];
                    *temp = (unsigned char) 0;
                }
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
