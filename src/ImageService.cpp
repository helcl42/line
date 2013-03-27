#include "ImageService.h"

ImageService::ImageService(std::vector<DetectedObject*>& shapes, DetectionSettings* settings)
: m_shrink(2), m_settings(settings), m_settingsIndex(0), m_lookUpLines(true)
{
    m_image = new Image<float>();
    m_colorImage = new Image<float>();
    m_lineDetector = new LineDetector(settings->getItem(0));      
    m_objectDetector = new SvgObjectDetector(shapes, settings->getItem(0));
}

ImageService::~ImageService()
{    
    SAFE_DELETE(m_objectDetector);
    SAFE_DELETE(m_lineDetector);
    SAFE_DELETE(m_colorImage);
    SAFE_DELETE(m_image);
}

Vector2<int>* ImageService::perform(const sensor_msgs::Image::ConstPtr& img, std::vector<float> cameraGroundAngles)
{
    unsigned long timeElapsed;
    Vector2<int>* objectPoint = NULL;
    IDetectedObject* object = NULL;

    m_shrinkTimer.start();

    m_image->setInstance(img, m_shrink);
    m_colorImage->setInstance(img, m_shrink);

//    m_lineDetector->invalidate();
//    m_lineDetector->initDetectionParams(m_shrink);
//    m_lineDetector->setImages(m_image, m_colorImage);
//    object = m_lineDetector->findObject();    

    m_objectDetector->invalidate();
    m_objectDetector->initDetectionParams(m_shrink);
    m_objectDetector->setImages(m_image, m_colorImage);
    m_objectDetector->setAngles(cameraGroundAngles);
    object = m_objectDetector->findObject();   

    if (object->isValid())
    {
        std::cout << "OBJECT!!! " << std::endl;
        m_changeColorTimer.stop();

        writeLinesToMessage(img, object->getPolygons(), object->getCountOfPolygons());

        if (objectPoint != NULL) SAFE_DELETE(objectPoint);

        objectPoint = object->getObjectPoint();
        std::cout << "POINT: " << *objectPoint << std::endl;
        if (objectPoint != NULL)
        {
            writePointToMessage(img, objectPoint);
        }
    }
    else
    {
        tryChangeSettings();
    }

//    for (unsigned int j = 0, k = 400; j < cameraGroundAngles.size(); j++, k += 27)
//    {
//        //m_image->writeCircle(k + 22, 22, 20, cameraGroundAngles[j]);
//        m_image->writeSquare(k + 48, 22, 25, 25, cameraGroundAngles[j], 20, 0);
//    }

    writeImageToMessage(img);

    m_shrinkTimer.stop();

    timeElapsed = m_shrinkTimer.getElapsedTimeInMicroSec();
    std::cout << "Elapsed " << timeElapsed << "ms " << m_shrinkTimer.getFPS() << " FPS" << std::endl;

    if (timeElapsed > 450000)
    {
        if (m_shrink < 4) m_shrink++;
    }
    else if (timeElapsed < 150000)
    {
        if (m_shrink > 1) m_shrink--;
    }

    std::cout << "Params: len = " << DetectionParams::minLineLengthTreshold << " Straight=  " << DetectionParams::maxStraightnessTreshold << std::endl;

    return objectPoint;
}

void ImageService::tryChangeSettings()
{
    if (!m_changeColorTimer.isStarted())
    {
        m_changeColorTimer.start();
    }
    else if (m_changeColorTimer.getElapsedTimeInMilliSec() > 5000) //pokud po 5 vterinach neuvidi hledanou caru, hleda dalsi
    {
        m_changeColorTimer.stop();
        std::cout << "Hledam dalsi !!" << std::endl;

        m_settingsIndex++;

        if (m_settingsIndex >= m_settings->getCountOfColors())
        {
            m_lookUpLines = !m_lookUpLines;
            m_settingsIndex = 0;
        }

        m_lineDetector->setColorSettings(m_settings->getItem(m_settingsIndex));
    }
}

void ImageService::writePointToMessage(const sensor_msgs::Image::ConstPtr& img, Vector2<int>* point, unsigned int size)
{
    unsigned char* temp;
    unsigned long imageSize = img->height * img->width * 3;
    unsigned int index;
    unsigned int min;
    unsigned int max;

    if (size <= 1)
    {
        min = 0;
        max = 1;
    }
    else
    {
        min = size / 2;
        max = size / 2;
    }

    for (unsigned int i = point->x - min; i < point->x + max; i++)
    {
        for (unsigned int j = point->y - min; j < point->y + max; j++)
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
                Vector2<int> point = oneLine->points[j];
                writePointToMessage(img, &point, width);
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
