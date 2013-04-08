#include "AbstractDetector.h"
#include "ImageFilters/ImageFilterFactory.h"

AbstractDetector::AbstractDetector(DetectionColorItem* settings)
: m_workImage(NULL), m_colorImage(NULL), m_settings(settings)
{    
    if (m_settings != NULL)
    {
        setBaseColor();
    }            
}

AbstractDetector::AbstractDetector(ImageMap<float>* image, Image<float>* colorImage)
: m_workImage(image), m_colorImage(colorImage), m_settings(NULL)
{            
}

AbstractDetector::~AbstractDetector()
{    
    SAFE_DELETE(m_imageFilterBatch);
}

void AbstractDetector::setBaseColor()
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

PixelRGB<float> AbstractDetector::getBaseColor() const
{
    return m_baseColor;
}

void AbstractDetector::setColorSettings(DetectionColorItem* settings)
{
    m_settings = settings;
    setBaseColor();
}

DetectionColorItem* AbstractDetector::getColorSettings() const
{
    return m_settings;
}

void AbstractDetector::setInstance(ImageMap<float>* image, Image<float>* colorImage)
{
    m_workImage = image;
    m_colorImage = colorImage;
}

void AbstractDetector::repaintSimilarColorPlaces(int interval)
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

void AbstractDetector::writeLineInImageMap(Line* line, unsigned int val)
{
    Vector2<int> linePoint;

    if (line == NULL) return;

    for (unsigned int i = 0; i < line->points.size(); i++)
    {
        linePoint = line->points[i];        
        m_workImage->setValueAt(linePoint.y, linePoint.x, val);                    
    }
}