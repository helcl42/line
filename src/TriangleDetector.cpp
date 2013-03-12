#include "TriangleDetector.h"

TriangleDetector::TriangleDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{
    this->initDetectionParams();
    m_bestTriangle = new Triangle();
}

TriangleDetector::TriangleDetector(Image<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{
    this->initDetectionParams();
    m_bestTriangle = new Triangle();
}

TriangleDetector::~TriangleDetector()
{
    SAFE_DELETE(m_bestTriangle);
}

StraightDetectedObject* TriangleDetector::findObject()
{
    m_edgeFilter->setImage(m_workImage);
    m_edgeFilter->applyFilter(DetectionParams::colorTreshold);
    m_imageFilter->setImage(m_workImage);
    m_imageFilter->gaussianBlur();
    traverseImage();
    return findBestTriangle();
}

void TriangleDetector::invalidate()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();
    m_bestTriangle->invalidate();
}

StraightDetectedObject* TriangleDetector::findBestTriangle()
{
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        m_bestTriangle->invalidate();
        lockAllLines(false);

        m_bestTriangle->setAt(m_lines[i], 0);

        lockSimilarLines(m_bestTriangle->getAt(0));
        m_bestTriangle->setAt(findLineWithDirection(m_bestTriangle->getAt(0)), 1);

        if (m_bestTriangle->isValid())
        {
            if(m_bestTriangle->getAt(0)->straightnessFactor > DetectionParams::maxStraightnessTreshold 
                    || m_bestTriangle->getAt(0)->straightnessFactor < DetectionParams::maxStraightnessTreshold / 2)
                continue;
            
            if(m_bestTriangle->getAt(1)->straightnessFactor > DetectionParams::maxStraightnessTreshold / 10) continue;            
            
            if (colorMatch())
            {
                std::cout << *m_bestTriangle << std::endl;
                writeLineInImage(m_bestTriangle->getAt(0), 255, 0, 0);
                writeLineInImage(m_bestTriangle->getAt(1), 0, 0, 255);
                break;
            }
        }
    }
    return m_bestTriangle;
}

bool TriangleDetector::colorMatch(unsigned int failCount)
{
    Pixel<float>* pixel = NULL;
    Vector2<int>* ret = NULL;
    
    Line* l1 = m_bestTriangle ->getAt(0);
    Line* l2 = m_bestTriangle->getAt(1);
    
    unsigned int count = 0;
    unsigned int len = l1->points.size() < l2->points.size() ? l1->points.size() : l2->points.size();

    for (unsigned int i = 0; i < len; i += DetectionParams::minLineLengthTreshold / 5)
    {
        ret = Vector2<int>::getPointBetween(l1->points[i], l2->points[i]);

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        {
            count++;
        }
        SAFE_DELETE(ret);
    }

    if (count > failCount)
    {
        return false;
    }

    return true;
}

void TriangleDetector::initDetectionParams(unsigned int shrink)
{
    unsigned int settingsParam = 480;

    DetectionParams::directionDeltaDegrees = 13;

    DetectionParams::minLineLengthTreshold = settingsParam / (shrink * 8);

    DetectionParams::maxLineLengthTreshold = settingsParam / (shrink * 2);

    DetectionParams::minStraightnessTreshold = 1;

    DetectionParams::maxStraightnessTreshold = pow(3, 0.5) * DetectionParams::maxLineLengthTreshold / 2;

    DetectionParams::minPointDistance = settingsParam / (shrink * 30);

    DetectionParams::maxPointDistance = pow(2 * DetectionParams::maxLineLengthTreshold * DetectionParams::maxLineLengthTreshold, 0.5);

    DetectionParams::inlineTolerance = settingsParam / (shrink * 20);

    DetectionParams::noCheckLineBorder = DetectionParams::minLineLengthTreshold / 10;

    DetectionParams::checkPointSkip = DetectionParams::minLineLengthTreshold / 6;

    DetectionParams::countOfDirections = 3; //index = count - 1       
}

