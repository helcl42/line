#include "LineDetector.h"
#include "ImageFilters/ImageFilterFactory.h"

LineDetector::LineDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{    
    this->initDetectionParams();    
    m_bestLine = new LinePair();
    this->m_imageFilterBatch = ImageFilterFactory<float>::createLineBatch();
}

LineDetector::LineDetector(ImageMap<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{    
    this->initDetectionParams();
    m_bestLine = new LinePair();
    this->m_imageFilterBatch = ImageFilterFactory<float>::createLineBatch();
}

LineDetector::~LineDetector()
{
    SAFE_DELETE(m_bestLine);
}

LinePair* LineDetector::findObject()
{
    //repaintSimilarColorPlaces();    
    m_imageFilterBatch->setInstance(m_workImage);
    m_imageFilterBatch->applyFilter();            
    
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
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {                
        m_bestLine->invalidate();
        lockAllLines(false);
        m_bestLine->setAt(m_lines[i], 0);

        lockSimilarLines(m_bestLine->getAt(0));
        m_bestLine->setAt(findLineWithDirection(m_bestLine->getAt(0)), 1);        

        if (m_bestLine->isValid())
        {            
            if (colorMatch() && m_bestLine->hasLengthInInterval())
            {                
                //std::cout << *m_bestLine << std::endl;                
                //writeLineInImageMap(m_bestLine->getAt(0), 255);
                //writeLineInImageMap(m_bestLine->getAt(1), 255); 
                break;
            }      
            else 
            {
                m_bestLine->invalidate();
            }
        }
    }
    return m_bestLine;
}

bool LineDetector::colorMatch(unsigned int failCount)
{
    Pixel<float>* pixel = NULL;
    Vector2<int>* ret = NULL;
    
    Polygon<int>* l1 = m_bestLine->getAt(0);
    Polygon<int>* l2 = m_bestLine->getAt(1);
    
    unsigned int count = 0;
    unsigned int len = l1->getSize() < l2->getSize() ? l1->getSize() : l2->getSize();

    for (unsigned int i = 0; i < len; i += DetectionParams::minLineLengthTreshold / 10)
    {
        ret = Vector2<int>::getPointBetween(l1->getPoint(i), l2->getPoint(i));

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        //if(Pixel<float>::colorDistance(&m_baseColor, pixel) > DetectionParams::colorTolerance)
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

void LineDetector::initDetectionParams(unsigned int shrink)
{
    unsigned int settingsParam = 480;         
    
    //DetectionParams::selectionTreshold = 8;
    
    DetectionParams::selectionTreshold = 25;
    
    DetectionParams::directionDeltaDegrees = 4;
    
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
