#include "TriangleDetector.h"


TriangleDetector::TriangleDetector(DetectionColorItem* settings)
: StraightObjectDetector(settings)
{
    this->setSettingsParam(240);
    this->initDetectionParams();
    m_bestTriangle = new Triangle();
}

TriangleDetector::TriangleDetector(Image<float>* image, Image<float>* colorImage)
: StraightObjectDetector(image, colorImage)
{
    this->setSettingsParam(240);
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
    m_bestTriangle->invalidate();
    sortLinesByStraightness();    

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        lockAllLines(false);
        
        //set first as default
        m_bestTriangle->setAt(m_lines[i], 0);
        
        //iterate trough all left needed lines
        for (unsigned int j = 1; j < 3; j++)
        {            
            //one cross allowed here
            lockSimilarLines(m_bestTriangle->getAt(j), 1);
            m_bestTriangle->setAt(findLineWithDirection(m_bestTriangle->getAt(j), 60), j);

            if (!m_bestTriangle->isValid(j)) break;

            m_bestTriangle->setLocked(j);
        }

        if (m_bestTriangle->isValid())
        {
            //if (lineColorMatch(rect[0], rect[2]))
            {
                writeLineInImage(m_bestTriangle->getAt(0), 255, 0, 0);
                writeLineInImage(m_bestTriangle->getAt(1), 0, 0, 255);
                writeLineInImage(m_bestTriangle->getAt(2), 255, 0, 0);
                writeLineInImage(m_bestTriangle->getAt(3), 0, 0, 255);
                break;
            }
        }
        else
        {
            std::cout << "fail!" << std::endl;
        }
    }
    return m_bestTriangle;
}

void TriangleDetector::initDetectionParams()
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

