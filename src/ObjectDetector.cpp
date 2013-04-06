#include "ObjectDetector.h"
#include "ImageFilters/ImageFilterFactory.h"

ObjectDetector::ObjectDetector(DetectionColorItem* settings)
: m_workImage(NULL), m_colorImage(NULL), m_settings(settings)
{    
    if (m_settings != NULL)
    {
        setBaseColor();
    }
    m_imageFilterBatch = ImageFilterFactory<float>::createBatch();
    
}

ObjectDetector::ObjectDetector(ImageMap<float>* image, Image<float>* colorImage)
: m_workImage(image), m_colorImage(colorImage), m_settings(NULL)
{    
    m_imageFilterBatch = ImageFilterFactory<float>::createBatch();
}

ObjectDetector::~ObjectDetector()
{    
    SAFE_DELETE(m_imageFilterBatch);
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

void ObjectDetector::setInstance(ImageMap<float>* image, Image<float>* colorImage)
{
    m_workImage = image;
    m_colorImage = colorImage;
}

void ObjectDetector::repaintSimilarColorPlaces(int interval)
{
    PixelRGB<float> pixelMinus;
    PixelRGB<float> pixelPlus;
    Pixel<float>* pixel = NULL;
    float value;
    
    for (int i = 0; i < 3; i++)
    {
        pixelMinus[i] = m_settings->color[i] > interval ? m_settings->color[i] - interval : 0;
        pixelPlus[i] = m_settings->color[i] + interval < 255 ? m_settings->color[i] + interval : 255;
    }

    for (unsigned int i = 0; i < m_workImage->getHeight(); ++i)
    {
        for (unsigned int j = 0; j < m_workImage->getWidth(); ++j)
        {
            pixel = m_colorImage->getPixel(i, j);

            if (pixel->isInInterval(&pixelMinus, &pixelPlus))
            {
                value = 255;
            }
            else
            {
                value = 0;
            }
            
            m_workImage->setValueAt(i, j, value);
        }
    }
}

void ObjectDetector::writeLineInImageMap(Line* line, unsigned int val)
{
    Vector2<int> linePoint;

    if (line == NULL) return;

    for (unsigned int i = 0; i < line->points.size(); i++)
    {
        linePoint = line->points[i];        
        m_workImage->setValueAt(linePoint.y, linePoint.x, val);                    
    }
}