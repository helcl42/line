#include "LineDetector.h"

LineDetector::LineDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{    
    this->initDetectionParams();    
    m_bestLine = new LinePair();
}

LineDetector::LineDetector(Image<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{    
    this->initDetectionParams();
    m_bestLine = new LinePair();
}

LineDetector::~LineDetector()
{
    SAFE_DELETE(m_bestLine);
}

StraightDetectedObject* LineDetector::findObject()
{
    //repaintSimilarColorPlaces();
    m_edgeFilter->setImage(m_workImage);
    m_edgeFilter->applyFilter(DetectionParams::colorTreshold);
    m_imageFilter->setImage(m_workImage);
    m_imageFilter->gaussianBlur();
    traverseImage();
    return findBestLine();
}

void LineDetector::invalidate()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();
    m_bestLine->invalidate();
}

StraightDetectedObject* LineDetector::findBestLine()
{    
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {        
        m_bestLine->invalidate();
        lockAllLines(false);
        m_bestLine->setAt(m_lines[i], 0);

        //if(ret->isTooNarrow()) continue;            
        
        lockSimilarLines(m_bestLine->getAt(0));
        m_bestLine->setAt(findLineWithDirection(m_bestLine->getAt(0)), 1);        

        if (m_bestLine->isValid())
        {
            //if(similar->isTooNarrow()) continue;
            
            if (lineColorMatch() && m_bestLine->hasLengthInInterval())
            {                
                std::cout << *m_bestLine << std::endl;                
                writeLineInImage(m_bestLine->getAt(0), 255, 0, 0);
                writeLineInImage(m_bestLine->getAt(0), 0, 0, 255); 
                break;
            }
            else
            {
                std::cout << "color fail!" << std::endl;
            }
        }
    }
    return m_bestLine;
}

bool LineDetector::lineColorMatch()
{
    Pixel<float>* pixel = NULL;
    Vector2<int>* ret = NULL;
    Line* l1 = m_bestLine->getAt(0);
    Line* l2 = m_bestLine->getAt(1);
    
    unsigned int failCount = 0;
    unsigned int len = l1->points.size() < l2->points.size() ? l1->points.size() : l2->points.size();

    for (unsigned int i = 0; i < len; i += 20)
    {
        ret = Vector2<int>::getPointBetween(l1->points[i], l2->points[i]);

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        {
            failCount++;
        }
        SAFE_DELETE(ret);
    }

    if (failCount > 0)
    {
        return false;
    }

    return true;
}

void LineDetector::initDetectionParams(unsigned int shrink)
{
    unsigned int settingsParam = 480;
     
    DetectionParams::directionDeltaDegrees = 10;
    
    DetectionParams::minLineLengthTreshold = settingsParam / (shrink * 2);
    
    DetectionParams::maxLineLengthTreshold = 2 * settingsParam;

    DetectionParams::minStraightnessTreshold = 0;
    
    DetectionParams::maxStraightnessTreshold = 3 * settingsParam / (shrink * 4);        

    DetectionParams::minPointDistance = settingsParam / (shrink * 30);

    DetectionParams::maxPointDistance = 2.5 * settingsParam / shrink;

    DetectionParams::inlineTolerance = settingsParam / (shrink * 60);

    DetectionParams::noCheckLineBorder = DetectionParams::minLineLengthTreshold / 3;

    DetectionParams::checkPointSkip = DetectionParams::minLineLengthTreshold / 10;

    DetectionParams::countOfDirections = 3; //index = count - 1   
}
