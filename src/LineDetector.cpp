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
    Line* ret = NULL;
    Line* similar = NULL;

    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {        
        lockAllLines(false);
        ret = m_lines[i];

        //if(ret->isTooNarrow()) continue;            
        
        lockSimilarLines(ret);
        similar = findLineWithDirection(ret);        

        if (ret != NULL && similar != NULL)
        {
            //if(similar->isTooNarrow()) continue;
            
            if (lineColorMatch(ret, similar) && ret->hasLengthInInterval(similar))
            {                
                std::cout << *ret << std::endl;
                std::cout << *similar << std::endl;
                writeLineInImage(ret, 255, 0, 0);
                writeLineInImage(similar, 0, 0, 255);
                m_bestLine->setAt(ret, 0);
                m_bestLine->setAt(similar, 1);
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

bool LineDetector::lineColorMatch(Line* l1, Line* l2)
{
    Pixel<float>* pixel = NULL;
    Vector2<int>* ret = NULL;
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
