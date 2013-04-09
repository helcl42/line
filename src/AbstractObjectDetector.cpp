#include "AbstractObjectDetector.h"

AbstractObjectDetector::AbstractObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings)
: AbstractDetector(settings), m_shapes(shapes), m_shrink(1)
{
    m_bestMatch = new GeneralObject();
    m_imageFilterBatch = ImageFilterFactory<float>::createShapeBatch();
}

AbstractObjectDetector::AbstractObjectDetector(std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage)
: AbstractDetector(image, colorImage), m_shapes(shapes), m_shrink(1)
{
    m_bestMatch = new GeneralObject();
    m_imageFilterBatch = ImageFilterFactory<float>::createShapeBatch();
}

AbstractObjectDetector::~AbstractObjectDetector()
{
}

void AbstractObjectDetector::setAngles(std::vector<float> angles)
{
    m_angles = angles;
}

void AbstractObjectDetector::setShrink(unsigned int shrink)
{
    m_shrink = shrink;
}

bool AbstractObjectDetector::colorMatch(unsigned int failCount)
{
    Vector2<int>* ret;
    Pixel<float>* pixel;
    Line<int>* line = m_bestMatch->getPolygon();
    unsigned int lenFourth = line->getSize() / 4;

    for (unsigned int i = 0; i < 2; i++)
    {
        ret = Vector2<int>::getPointBetween(line->points[i * lenFourth], line->points[(i + 2) * lenFourth]);

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        SAFE_DELETE(ret);
        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        {
            return false;
        }
    }
    return true;
}
