#include "RectangleDetector.h"

RectangleDetector::RectangleDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{    
    m_bestRectangle = new Rectangle();
}


RectangleDetector::RectangleDetector(Image<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{   
    m_bestRectangle = new Rectangle();
}


RectangleDetector::~RectangleDetector()
{    
    SAFE_DELETE(m_bestRectangle);
}


Rectangle* RectangleDetector::detectRectangle()
{
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


Rectangle* RectangleDetector::findBestRectangle()
{
    Line* rect[4];

    for(int i = 0; i < 4; i++)
    {
        rect[i] = NULL;
    }
    
    //sortLinesByStraightness();
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        lockAllLines(false);
        rect[0] = m_lines[i];

        lockSimilarLines(rect[0]);        
        rect[1] = findLineWithDirection(rect[0], 90);

        if (rect[0] != NULL && rect[1] != NULL)
        {
            rect[0]->locked = true;
            rect[1]->locked = true;

            rect[2] = findLineWithDirection(rect[1]);

            if (rect[2] != NULL)
            {
                rect[2]->locked = true;
                
                rect[3] = findLineWithDirection(rect[1], 90);
                if(rect[3] != NULL)
                    
                //if (lineColorMatch(rect[0], rect[2]))
                {                    
                    writeLineInImage(rect[0], 255, 0, 0);
                    writeLineInImage(rect[1], 0, 0, 255);
                    writeLineInImage(rect[2], 255, 0, 0);
                    writeLineInImage(rect[3], 0, 0, 255);                    
                    break;
                }
            }
        }
        else
        {
            std::cout << "fail!" << std::endl;
        }
    }
    return m_bestRectangle;
}


void RectangleDetector::initDetectionParams()
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
