#include "RectangleDetector.h"

RectangleDetector::RectangleDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{
    this->setSettingsParam(240);
    this->initDetectionParams();
    m_bestRectangle = new Rectangle();
}

RectangleDetector::RectangleDetector(Image<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{
    this->setSettingsParam(240);
    this->initDetectionParams();
    m_bestRectangle = new Rectangle();
}

RectangleDetector::~RectangleDetector()
{
    SAFE_DELETE(m_bestRectangle);
}

StraightDetectedObject* RectangleDetector::findObject()
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

StraightDetectedObject* RectangleDetector::findBestRectangle()
{
    m_bestRectangle->invalidate();
    sortLinesByStraightness();    

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        lockAllLines(false);
        
        //set first as default
        m_bestRectangle->setAt(m_lines[i], 0);
        
        //iterate trough all left needed lines
        for (unsigned int j = 1; j < 4; j++)
        {            
            //one cross allowed here
            lockSimilarLines(m_bestRectangle->getAt(j), 1);
            m_bestRectangle->setAt(findLineWithDirection(m_bestRectangle->getAt(j), 90), j);

            if (!m_bestRectangle->isValid(j)) break;

            m_bestRectangle->setLocked(j);
        }

        if (m_bestRectangle->isValid())
        {
            //if (lineColorMatch(rect[0], rect[2]))
            {
                writeLineInImage(m_bestRectangle->getAt(0), 255, 0, 0);
                writeLineInImage(m_bestRectangle->getAt(1), 0, 0, 255);
                writeLineInImage(m_bestRectangle->getAt(2), 255, 0, 0);
                writeLineInImage(m_bestRectangle->getAt(3), 0, 0, 255);
                break;
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
    DetectionParams::lineLengthTreshold = 80;

    DetectionParams::straightnessTreshold = 3;

    DetectionParams::minPointDistance = 40;

    DetectionParams::maxPointDistance = 600;

    DetectionParams::inlineTolerance = 20;

    DetectionParams::noCheckLineBorder = 80;

    DetectionParams::checkPointSkip = 10;

    DetectionParams::countOfDirections = 3; //index = count - 1

    DetectionParams::imageHeight = 480;

    DetectionParams::imageWidth = 640;
}
