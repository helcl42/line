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
    //repaintSimilarColorPlaces();
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
    Line* ret = NULL;
    Line* similar = NULL;
    
    //sortLinesByStraightness(true);    
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {        
        lockAllLines(false);
        ret = m_lines[i];   
        
        lockSimilarLines(ret);
        similar = findLineWithDirection(ret);        

        if (ret != NULL && similar != NULL)
        {                        
            if (/*lineColorMatch(ret, similar) &&*/ ret->hasLengthInInterval(similar, 5))
            {                
                std::cout << *ret << std::endl;
                std::cout << *similar << std::endl;
                writeLineInImage(ret, 255, 0, 0);
                writeLineInImage(similar, 0, 0, 255);
                m_bestRectangle->setAt(ret, 0);
                m_bestRectangle->setAt(similar, 1);
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
