#include "SvgObjectDetector.h"
#include "Shapes/GeneralObject.h"
#include "Utils/Timer.h"
#include "ImageService.h"

SvgObjectDetector::SvgObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings)
: ObjectDetector(settings), m_shapes(shapes), m_shrink(1)
{
    this->initDetectionParams();
    m_bestMatch = new GeneralObject();
}

SvgObjectDetector::SvgObjectDetector(std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage)
: ObjectDetector(image, colorImage), m_shapes(shapes), m_shrink(1)
{
    this->initDetectionParams();
    m_bestMatch = new GeneralObject();
}

SvgObjectDetector::~SvgObjectDetector()
{
    m_shapes.clear();
    SAFE_DELETE(m_bestMatch);
}

void SvgObjectDetector::invalidate()
{
    m_bestMatch->invalidate();
}

void SvgObjectDetector::setAngles(std::vector<float> angles)
{
    m_angles = angles;

    //    for(unsigned int i = 0; i < m_angles.size(); i++)
    //    {
    //        m_angles.push_back(-m_angles[i]);
    //    }    
}

void SvgObjectDetector::setShrink(unsigned int shrink)
{
    m_shrink = shrink;
}

void SvgObjectDetector::cleanUp()
{
    std::vector<DetectedObject*>::iterator ii;
    for (ii = m_detectedShapes.begin(); ii != m_detectedShapes.end(); ++ii)
    {
        SAFE_DELETE(*ii);
    }
    m_detectedShapes.clear();

    m_bestMatch->invalidate();
}

void SvgObjectDetector::generateShapes(unsigned int shapeIndex, unsigned int size)
{
    DetectedObject* tempShapePtr;
    DetectedObject* shape = m_shapes[shapeIndex];

    for (int rotateAngle = -90; rotateAngle < 90; rotateAngle += 4)
    {
        tempShapePtr = new GeneralObject(new Line(shape->getPolygon()));        

        if (rotateAngle > 0)
        {
            tempShapePtr->createBatch(size, rotateAngle, true);
        }
        else
        {
            tempShapePtr->createBatch(size, rotateAngle, false);
        }       

        m_detectedShapes.push_back(tempShapePtr);
    }
}

DetectedObject* SvgObjectDetector::findObject()
{
    cleanUp();
    //repaintSimilarColorPlaces();        
    m_imageFilterBatch->setInstance(m_workImage);
    m_imageFilterBatch->applyFilter();
    //m_workImage->resolveThreshold(150);
    return findBestShape();
}

bool SvgObjectDetector::rawShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    unsigned int ratio = 16, failCount = 0;
    double percentFail;
    Line* squareLine = shape->getPolygon();
    unsigned int lineSize = squareLine->getSize() / ratio;

    for (unsigned int k = 0; k < lineSize; k += ratio)
    {
        Vector2<int> point = squareLine->points[k];
        if (m_workImage->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold) failCount++;
    }

    percentFail = (double) failCount / (double) lineSize;

    if (percentFail < 0.05) return true;
    return false;
}

bool SvgObjectDetector::innerShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    Line* shapeLine = shape->getPolygon();
    unsigned int lineSize = shapeLine->getSize();
    unsigned int failCount = 0;
    double percentFail;
    Vector2<int>* point;    

    if (!rawShapeFind(shape, y, x)) return false;

    for (unsigned int k = 0; k < lineSize; k++)
    {
        point = shapeLine->getPointPtr(k);
        if (m_workImage->getValueAt(point->y + y, point->x + x) < DetectionParams::selectionTreshold) failCount++;    
    }
    
    percentFail = (double) failCount / (double) lineSize;

    if (percentFail < DetectionParams::maxPercentageError)
    {        
        std::cout << "failCount = " << failCount << " size = " << lineSize << " fail = " << percentFail << "%" << std::endl;
        m_bestMatch->cleanUp();
        for (unsigned int k = 0; k < lineSize; k++)
        {
            point = shapeLine->getPointPtr(k);
            m_bestMatch->addPoint(point->x + x, point->y + y);
        }

        return true;
    }
    return false;
}

bool SvgObjectDetector::findShapeInImagePart(DetectedObject* shape)
{
    unsigned int baseHeight = m_workImage->getHeight();
    unsigned int baseWidth = m_workImage->getWidth();
    unsigned int offsetX = shape->getWidth();
    unsigned int offsetY = shape->getHeight();
    //unsigned int angle;

    std::vector<float>::reverse_iterator anglesIterator;
    for (anglesIterator = m_angles.rbegin(); anglesIterator != m_angles.rend(); ++anglesIterator)
    {        
        shape->viewByAngle(*anglesIterator);        

        for (unsigned int i = 0; i < baseHeight - offsetY - 1; i += 3)
        {
            for (unsigned int j = 0; j < baseWidth - offsetX - 1; j += 3)
            {
                if (innerShapeFind(shape, i, j))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

DetectedObject* SvgObjectDetector::findBestShape()
{
    unsigned int shapeIndex = 0;
    unsigned int shapeSize = m_workImage->getHeight() / 3 - 1;

    while (shapeSize > m_workImage->getHeight() / 5)
    {
        cleanUp();

        generateShapes(shapeIndex, shapeSize);

        for (unsigned int i = 0; i < m_detectedShapes.size(); i++)
        {
            if (findShapeInImagePart(m_detectedShapes[i]))
            {
                //                if (colorMatch())
                //                {
                std::cout << "GOT IT!!!!" << std::endl;
                //writeLineInImageMap(m_bestMatch->getPolygon(), 255);
                ImageService::getInstance()->writeLL(m_bestMatch->getPolygon());
                return m_bestMatch;
                //                }                
            }
        }        
        invalidate();

        shapeSize -= 5;
    }
    invalidate();   
    return m_bestMatch;
}

bool SvgObjectDetector::colorMatch(unsigned int failCount)
{
    Vector2<int>* ret;
    Pixel<float>* pixel;
    Line* line = m_bestMatch->getPolygon();
    unsigned int lenFourth = line->getSize() / 4;

    for (unsigned int i = 0; i < 2; i++)
    {
        ret = Vector2<int>::getPointBetween(line->points[i * lenFourth], line->points[(i + 2) * lenFourth]);

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        SAFE_DELETE(ret);
        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        {
            return false;
        }
    }
    return true;
}

void SvgObjectDetector::initDetectionParams(unsigned int shrink)
{
    DetectionParams::selectionTreshold = 35;

    DetectionParams::maxPercentageError = 0.05;
}

