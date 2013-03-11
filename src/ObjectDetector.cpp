#include "ObjectDetector.h"
#include "SobelFilterStrategy.h"

ObjectDetector::ObjectDetector(DetectionColorItem* settings)
: m_workImage(NULL), m_colorImage(NULL), m_settings(settings)
{
    m_imageFilter = new ImageFilter<float>();
    m_edgeFilter = new SobelFilterStrategy<float>();
    if (m_settings != NULL)
    {
        setBaseColor();
    }
}

ObjectDetector::ObjectDetector(Image<float>* image, Image<float>* colorImage)
: m_workImage(image), m_colorImage(colorImage), m_settings(NULL)
{
    m_imageFilter = new ImageFilter<float>();
    m_edgeFilter = new SobelFilterStrategy<float>();
}

ObjectDetector::~ObjectDetector()
{
    SAFE_DELETE(m_edgeFilter);
    SAFE_DELETE(m_imageFilter);
}

void ObjectDetector::setBaseColor()
{
    float maxValue = 0;
    int maxIndex = 0;
    float temp;

    for (int i = 0; i < 3; i++)
    {
        temp = m_settings->color[i];
        if (temp > maxValue)
        {
            maxValue = temp;
            maxIndex = i;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (i == maxIndex)
        {
            m_baseColor[i] = 255;
        }
        else
        {
            m_baseColor[i] = 0;
        }
    }
}

PixelRGB<float> ObjectDetector::getBaseColor() const
{
    return m_baseColor;
}

void ObjectDetector::setColorSettings(DetectionColorItem* settings)
{
    m_settings = settings;
    setBaseColor();
}

DetectionColorItem* ObjectDetector::getColorSettings() const
{
    return m_settings;
}

void ObjectDetector::setImages(Image<float>* image, Image<float>* colorImage)
{
    m_workImage = image;
    m_colorImage = colorImage;
}

void ObjectDetector::replaintSimilarColorPlaces(int interval)
{
    PixelRGB<float> pixelMinus;
    PixelRGB<float> pixelPlus;
    Pixel<float>* pixel = NULL;

    for (int i = 0; i < 3; i++)
    {
        pixelMinus[i] = m_settings->color[i] > interval ? m_settings->color[i] - interval : 0;
        pixelPlus[i] = m_settings->color[i] + interval < 255 ? m_settings->color[i] + interval : 255;
    }

    for (unsigned int i = 0; i < m_workImage->getHeight(); ++i)
    {
        for (unsigned int j = 0; j < m_workImage->getWidth(); ++j)
        {
            pixel = m_workImage->getPixel(i, j);

            if (pixel->isInInterval(&pixelMinus, &pixelPlus))
            {
                pixel->r = m_baseColor.r;
                pixel->g = m_baseColor.g;
                pixel->b = m_baseColor.b;
            }
            else
            {
                pixel->r = 0;
                pixel->g = 0;
                pixel->b = 0;
            }
        }
    }
}
