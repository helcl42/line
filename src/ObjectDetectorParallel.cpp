#include "ObjectDetectorParallel.h"

#include "Shapes/GeneralObject.h"
#include "Utils/Timer.h"
#include "ImageService.h"
#include "ImageFilters/ImageFilterFactory.h"

ObjectDetectorParallel::ObjectDetectorParallel(unsigned int numberOfInstances, std::vector<DetectedObject*>& shapes, DetectionColorItem* settings)
: AbstractObjectDetector(shapes, settings)
{
    this->initDetectionParams();
    for (unsigned int i = 0; i < numberOfInstances; i++)
    {
        m_workers.push_back(new ObjectDetectorThread());
    }
    m_shapesCache = new ShapesCache();
}

ObjectDetectorParallel::ObjectDetectorParallel(unsigned int numberOfInstances, std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage)
: AbstractObjectDetector(shapes, image, colorImage)
{
    this->initDetectionParams();
    for (unsigned int i = 0; i < numberOfInstances; i++)
    {
        m_workers.push_back(new ObjectDetectorThread());
    }
    m_shapesCache = new ShapesCache();
}

ObjectDetectorParallel::~ObjectDetectorParallel()
{
    SAFE_DELETE(m_shapesCache);
    SAFE_DELETE(m_bestMatch);
}

void ObjectDetectorParallel::invalidate()
{
    m_bestMatch->invalidate();
}

void ObjectDetectorParallel::cleanUp()
{
    std::vector<DetectedObject*>::iterator ii;
//    for (ii = m_detectedShapes.begin(); ii != m_detectedShapes.end(); ++ii)
//    {
//        SAFE_DELETE(*ii);
//    }
    m_detectedShapes.clear();

    m_bestMatch->invalidate();
}

void ObjectDetectorParallel::generateShapes(DetectedObject* shape, unsigned int size)
{
    DetectedObject* tempShapePtr;

    for (int rotateAngle = -90; rotateAngle < 90; rotateAngle += 4)
    {
        for (unsigned int viewAngle = 0; viewAngle < m_angles.size(); viewAngle++)
        {
            tempShapePtr = new GeneralObject(new Polygon<int>(shape->getPolygon()));

            if (rotateAngle > 0)
            {
                tempShapePtr->createBatch(size, rotateAngle, true);
            }
            else
            {
                tempShapePtr->createBatch(size, rotateAngle, false);
            }

            //optimize !!! shifting
            tempShapePtr->viewByAngle(m_angles[viewAngle], true);

            m_detectedShapes.push_back(tempShapePtr);
        }
    }
}

void ObjectDetectorParallel::generate(unsigned int shapeIndex)
{
    DetectedObject* shape = m_shapes[shapeIndex];
    unsigned int shapeSize = m_workImage->getHeight() / 2 - 1;

    while (shapeSize > m_workImage->getHeight() / 6)
    {
        cleanUp();

        generateShapes(shape, shapeSize);

        shapeSize -= 4;
    }
}

DetectedObject* ObjectDetectorParallel::findObject()
{
    unsigned int partSize;
    unsigned int shapeIterator = 0;
    int tempCount = m_detectedShapes.size();

    m_imageFilterBatch->setInstance(m_workImage);
    m_imageFilterBatch->applyFilter();
    //m_workImage->resolveThreshold(20);

    cleanUp();
    
    if (m_shapesCache->itemExists(m_shrink))
    {     
        m_detectedShapes = m_shapesCache->getItem(m_shrink);
    }
    else
    {                
        generate(0);
        m_shapesCache->addItem(m_shrink, m_detectedShapes);
    }

    partSize = tempCount / m_workers.size();

    std::vector<ObjectDetectorThread*>::iterator ii;
    for (ii = m_workers.begin(); ii != m_workers.end(); ++ii)
    {
        std::vector<DetectedObject* > objects;

        for (unsigned int j = 0; j < partSize; j++, shapeIterator++)
        {
            objects.push_back(m_detectedShapes[shapeIterator]);
        }

        tempCount -= partSize;

        if (tempCount < partSize)
        {
            for (; shapeIterator < m_detectedShapes.size(); shapeIterator++)
            {
                objects.push_back(m_detectedShapes[shapeIterator]);
            }
        }

        (*ii)->setInstance(m_workImage, objects);
    }

    return findBestShape();
}

DetectedObject* ObjectDetectorParallel::findBestShape()
{
    invalidate();

    for (unsigned int i = 0; i < m_workers.size(); i++)
    {
        m_workers[i]->runThread();
    }

    for (unsigned int i = 0; i < m_workers.size(); i++)
    {
        //std::cout << "JOIN " << i << std::endl;
        m_workers[i]->waitForThreadToExit();
    }

    for (unsigned int i = 0; i < m_workers.size(); i++)
    {
        if (m_workers[i]->found())
        {
            if (m_workers[i]->getFoundObject()->isValid())
            {
                //color match            
                //if(colorMatch()) {                    
                ImageService::getInstance()->writeLL(m_workers[i]->getFoundObject()->getPolygon());
                std::cout << "THREAD " << i << " FOUND IT !!!!!!!!!!!!!!" << std::endl;
                //}
            }
        }
    }

    return m_bestMatch;
}

void ObjectDetectorParallel::initDetectionParams(unsigned int shrink)
{
    //big gauss kernel
//    DetectionParams::selectionTreshold = 25;
//
//    DetectionParams::maxPercentageError = 0.05;
    
    //small kernel
//    DetectionParams::selectionTreshold = 35;
//
//    DetectionParams::maxPercentageError = 0.1;        
    
    //big sobel
    DetectionParams::selectionTreshold = 10;

    DetectionParams::maxPercentageError = 0.1;
}
