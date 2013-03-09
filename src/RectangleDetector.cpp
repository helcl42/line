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
    m_bestRectangle->invalidate();
    //sortLinesByStraightness();
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        lockAllLines(false);
        
        for (unsigned int j = 0; j < 4; j++)
        {
            m_bestRectangle->setAt(m_lines[i], j);
            lockSimilarLines(m_bestRectangle->getAt(j));
            m_bestRectangle->setAt(findLineWithDirection(m_bestRectangle->getAt(j), 90), j);

            if (!m_bestRectangle->isValid(j + 1)) break;

            m_bestRectangle->setLocked(j + 1);
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
