#include "RectangleDetector.h"

RectangleDetector::RectangleDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{ 
    this->initDetectionParams();
    m_bestRectangle = new Rectangle();
}

RectangleDetector::RectangleDetector(Image<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{    
    this->initDetectionParams();
    m_bestRectangle = new Rectangle();
}

RectangleDetector::~RectangleDetector()
{
    SAFE_DELETE(m_bestRectangle);
}

StraightDetectedObject* RectangleDetector::findObject()
{
    //gitrepaintSimilarColorPlaces();
    m_edgeFilter->setImage(m_workImage);
    m_edgeFilter->applyFilter(DetectionParams::colorTreshold);
    m_imageFilter->setImage(m_workImage);
    m_imageFilter->gaussianBlur();
    traverseImage();
    return findBestRectangle();
}

void RectangleDetector::invalidate()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();
    m_bestRectangle->invalidate();
}

StraightDetectedObject* RectangleDetector::findBestRectangle()
{                
    sortLinesByStraightness(true);    
    //sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {        
        m_bestRectangle->invalidate();
        lockAllLines(false);
        
        m_bestRectangle->setAt(m_lines[i], 0);
        
        lockSimilarLines(m_bestRectangle->getAt(0));
        m_bestRectangle->setAt(findLineWithDirection(m_bestRectangle->getAt(0)), 1);

        if (m_bestRectangle->isValid())
        {                        
            if (/*lineColorMatch(ret, similar) &&*/ m_bestRectangle->hasLengthInInterval(5))
            {                
                std::cout << *m_bestRectangle << std::endl;
                writeLineInImage(m_bestRectangle->getAt(0), 255, 0, 0);
                writeLineInImage(m_bestRectangle->getAt(1), 0, 0, 255);                
                break;
            }            
        }
    }
    return m_bestRectangle;
}

void RectangleDetector::initDetectionParams(unsigned int shrink)
{
    unsigned int settingsParam = 480;
     
    DetectionParams::directionDeltaDegrees = 13;
        
    DetectionParams::minLineLengthTreshold = settingsParam / (shrink * 8);
    
    DetectionParams::maxLineLengthTreshold = settingsParam / (shrink * 4);

    DetectionParams::minStraightnessTreshold = pow(2, 0.5) * DetectionParams::minLineLengthTreshold / 4;
    
    DetectionParams::maxStraightnessTreshold = pow(2, 0.5) * DetectionParams::maxLineLengthTreshold;

    DetectionParams::minPointDistance = settingsParam / (shrink * 30);

    DetectionParams::maxPointDistance = pow(2 * DetectionParams::maxLineLengthTreshold * DetectionParams::maxLineLengthTreshold, 0.5);

    DetectionParams::inlineTolerance = settingsParam / (shrink * 20);

    DetectionParams::noCheckLineBorder = DetectionParams::minLineLengthTreshold / 10;

    DetectionParams::checkPointSkip = DetectionParams::minLineLengthTreshold / 6;

    DetectionParams::countOfDirections = 3; //index = count - 1       
}
