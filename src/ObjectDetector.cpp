#include "ObjectDetector.h"
#include "Shapes/GeneralObject.h"
#include "Utils/Timer.h"
#include "ImageService.h"
#include "ImageFilters/ImageFilterFactory.h"

ObjectDetector::ObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings)
: AbstractObjectDetector(shapes, settings)
{
    this->initDetectionParams();        
}

ObjectDetector::ObjectDetector(std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage)
: AbstractObjectDetector(shapes, image, colorImage)
{
    this->initDetectionParams();    
}

ObjectDetector::~ObjectDetector()
{
    m_shapes.clear();
    SAFE_DELETE(m_bestMatch);
}

void ObjectDetector::invalidate()
{
    m_bestMatch->invalidate();
}

void ObjectDetector::cleanUp()
{
    std::vector<DetectedObject*>::iterator ii;
    for (ii = m_detectedShapes.begin(); ii != m_detectedShapes.end(); ++ii)
    {
        SAFE_DELETE(*ii);
    }
    m_detectedShapes.clear();

    m_bestMatch->invalidate();
}

void ObjectDetector::generateShapes(DetectedObject* shape, unsigned int size)
{
    DetectedObject* tempShapePtr;    

    for (int rotateAngle = -90; rotateAngle < 90; rotateAngle += 4)
    {
        tempShapePtr = new GeneralObject(new Line<int>(shape->getPolygon()));

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

DetectedObject* ObjectDetector::findObject()
{
    cleanUp();
    //repaintSimilarColorPlaces();        
    m_imageFilterBatch->setInstance(m_workImage);
    m_imageFilterBatch->applyFilter();
    //m_workImage->resolveThreshold(150);

    return findBestShape();
}

bool ObjectDetector::rawShapeFind(DetectedObject* shape, unsigned int y, unsigned int x, unsigned int ratio, unsigned int base)
{
    double percentFail;
    Line<int>* squareLine = shape->getPolygon();
    unsigned int lineSize = squareLine->getSize();
    unsigned int failCount = 0;

    for (unsigned int k = base; k < lineSize; k += ratio)
    {
        Vector2<int> point = squareLine->points[k];
        if (m_workImage->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold) failCount++;
    }

    percentFail = (double) failCount / (double) lineSize / ratio;

    if (percentFail < DetectionParams::maxPercentageError) return true;
    return false;
}

bool ObjectDetector::innerShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    Line<int>* shapeLine = shape->getPolygon();
    unsigned int lineSize = shapeLine->getSize();    
    Vector2<int>* point;    
    unsigned int i = 16, j = 16, iteration = 0;

    while (rawShapeFind(shape, y, x, i, j))
    {
        if (iteration != 0) i >>= 1;
        j = i >> 1;

        if(i == 0) break;
        
        iteration++;
    }

    if (i <= 1)
    {        
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

bool ObjectDetector::findShapeInImage(DetectedObject* shape)
{
    unsigned int baseHeight = m_workImage->getHeight();
    unsigned int baseWidth = m_workImage->getWidth();
    unsigned int offsetX = shape->getWidth();
    unsigned int offsetY = shape->getHeight();    

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

DetectedObject* ObjectDetector::findBestShape()
{
    unsigned int shapeIndex = 0;
    unsigned int shapeSize = m_workImage->getHeight() / 3 - 1;
    
    while (shapeSize > m_workImage->getHeight() / 5)
    {
        cleanUp();

        generateShapes(m_shapes[shapeIndex], shapeSize);

        for (unsigned int i = 0; i < m_detectedShapes.size(); i++)
        {
            if (findShapeInImage(m_detectedShapes[i]))
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

void ObjectDetector::initDetectionParams(unsigned int shrink)
{
    DetectionParams::selectionTreshold = 10;

    DetectionParams::maxPercentageError = 0.01;
}

