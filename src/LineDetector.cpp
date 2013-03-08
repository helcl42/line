#include "LineDetector.h"
#include "CannyFilterStategy.h"

LineDetector::LineDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{    
    m_bestLine = new LinePair();
}

LineDetector::LineDetector(Image<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{   
    m_bestLine = new LinePair();
}

LineDetector::~LineDetector()
{    
    SAFE_DELETE(m_bestLine);    
}

LinePair* LineDetector::detectLine()
{
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


LinePair* LineDetector::findBestLine()
{
    Line* ret = NULL;
    Line* similar = NULL;

    //sortLinesByStraightness();
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        lockAllLines(false);
        ret = m_lines[i];

        lockSimilarLines(ret);        
        similar = findLineWithDirection(ret);

        if (ret != NULL && similar != NULL)
        {
            if (lineColorMatch(ret, similar))
            {                
                std::cout << *ret << std::endl;
                std::cout << *similar << std::endl;
                writeLineInImage(ret, 255, 0, 0);
                writeLineInImage(similar, 0, 0, 255);
                m_bestLine->setLine(ret, similar);
                break;
            }
        }
        else
        {
            std::cout << "fail!" << std::endl;
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


void LineDetector::initDetectionParams()
{            
    DetectionParams::lineLengthTreshold = 240;

    DetectionParams::straightnessTreshold = 360;

    DetectionParams::minPointDistance = 16;

    DetectionParams::maxPointDistance = 1300;

    DetectionParams::inlineTolerance = 8;

    DetectionParams::noCheckLineBorder = 80;

    DetectionParams::checkPointSkip = 24;

    DetectionParams::countOfDirections = 3; //index = count - 1

    DetectionParams::imageHeight = 480;

    DetectionParams::imageWidth = 640;
}
