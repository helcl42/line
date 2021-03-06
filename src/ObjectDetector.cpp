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

    for (int rotateAngle = 0; rotateAngle < 360; rotateAngle += 10)
    {
        for (unsigned int viewAngle = 0; viewAngle < m_angles.size(); viewAngle++)
        {
            tempShapePtr = new GeneralObject(new Polygon<int>(shape->getPolygon()));

            tempShapePtr->createBatch(size, rotateAngle);

            tempShapePtr->viewByAngle(m_angles[viewAngle], true);

            m_detectedShapes.push_back(tempShapePtr);
        }
    }
}

DetectedObject* ObjectDetector::findObject()
{
    cleanUp();
    //repaintSimilarColorPlaces();        
    m_imageFilterBatch->setInstance(m_workImage);
    m_imageFilterBatch->applyFilter();

    m_workImage->increasePower(4, 1);

    return findBestShape();
}

bool ObjectDetector::rawShapeFind(DetectedObject* shape, unsigned int y, unsigned int x, unsigned int ratio, unsigned int base)
{
    double percentFail;
    Vector2<int>* point;
    Polygon<int>* squareLine = shape->getPolygon();
    unsigned int lineSize = squareLine->getSize();
    unsigned int failCount = 0;

    for (unsigned int k = base; k < lineSize; k += ratio)
    {
        point = squareLine->getPointPtr(k);
        if (m_workImage->getValueAt(point->y + y, point->x + x) < DetectionParams::selectionTreshold) failCount++;
    }

    percentFail = (double) failCount / ((double) lineSize / (double) ratio);

    if (percentFail < DetectionParams::maxPercentageError) return true;
    return false;
}

bool ObjectDetector::innerShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    Vector2<int>* point;
    Polygon<int>* shapeLine = shape->getPolygon();
    unsigned int lineSize = shapeLine->getSize();
    
    unsigned int ratio = Utils::computePower2Of(shapeLine->getSize() / 5, false);    
    unsigned int i = ratio, j = ratio, iteration = 0;

    while (rawShapeFind(shape, y, x, i, j))
    {
        if (iteration != 0) i >>= 1;
        j = i >> 1;

        if (i == 1) break;

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

    for (unsigned int i = 0; i < baseHeight - offsetY - 1; i += 4)
    {
        for (unsigned int j = 0; j < baseWidth - offsetX - 1; j += 4)
        {
            if (innerShapeFind(shape, i, j))
            {
                return true;
            }
        }
    }
    return false;
}

DetectedObject* ObjectDetector::findBestShape()
{
    unsigned int shapeIndex = 0;
    unsigned int shapeSize = m_workImage->getHeight() / 2 - 1;

    unsigned int step = m_workImage->getHeight() / 16; //32;
    if(step == 0) step = 1;    
    
    while (shapeSize > m_workImage->getHeight() / 4)
    {
        cleanUp();

        generateShapes(m_shapes[shapeIndex], shapeSize);

        for (unsigned int i = 0; i < m_detectedShapes.size(); i++)
        {
            if (findShapeInImage(m_detectedShapes[i]))
            {
                // if (colorMatch())
                // {
                std::cout << "GOT IT!!!!" << std::endl;                
                return m_bestMatch;
                // }                
            }
        }
        invalidate();
        shapeSize -= step;
    }
    invalidate();
    return m_bestMatch;
}

void ObjectDetector::initDetectionParams(unsigned int shrink)
{
    DetectionParams::selectionTreshold = 10;

    DetectionParams::maxPercentageError = 0.1;
}

