#include "ObjectDetector.h"
#include "SobelFilterStrategy.h"

ObjectDetector::ObjectDetector(DetectionColorItem* settings)
: m_workImage(NULL), m_colorImage(NULL), m_settings(settings), m_imageMap(NULL)
{
    m_imageFilter = new ImageFilter<float>();
    m_edgeFilter = new SobelFilterStrategy<float>();
    if (m_settings != NULL)
    {
        setBaseColor();
    }
    m_imageMap = new ImageMap<unsigned int>();
}

ObjectDetector::ObjectDetector(Image<float>* image, Image<float>* colorImage)
: m_workImage(image), m_colorImage(colorImage), m_settings(NULL), m_imageMap(NULL)
{
    m_imageFilter = new ImageFilter<float>();
    m_edgeFilter = new SobelFilterStrategy<float>();
    m_imageMap = new ImageMap<unsigned int>(image);
}

ObjectDetector::~ObjectDetector()
{
    SAFE_DELETE(m_edgeFilter);
    SAFE_DELETE(m_imageFilter);
    SAFE_DELETE(m_imageMap);
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

void ObjectDetector::repaintSimilarColorPlaces(int interval)
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

void ObjectDetector::writeLineInImage(Line* line, int r, int g, int b)
{
    Vector2<int> linePoint;

    if (line == NULL) return;

    for (unsigned int i = 0; i < line->points.size(); i++)
    {
        linePoint = line->points[i];
        if (i < 15)
        {
            m_workImage->setPixelValue(linePoint.y, linePoint.x, 0, 255, 255);
        }
        else
        {
            m_workImage->setPixelValue(linePoint.y, linePoint.x, r, g, b);
        }
    }
}