/* 
 * File:   SvgObjectDetectorParallel.h
 * Author: lubos
 *
 * Created on April 8, 2013, 8:23 PM
 */

#ifndef OBJECTDETECTORPARALLEL_H
#define	OBJECTDETECTORPARALLEL_H
/* 
 * File:   SquareDetector.h
 * Author: lubos
 *
 * Created on March 18, 2013, 4:00 AM
 */
#include "AbstractDetector.h"
#include "Shapes/DetectedObject.h"
#include "Threading/Thread.h"
#include "ObjectDetectorThread.h"
#include "AbstractObjectDetector.h"


class ObjectDetectorParallel : public AbstractObjectDetector
{
protected:            
    std::vector<ObjectDetectorThread*> m_workers;   
        
    ShapesCache* m_shapesCache;

public:
    ObjectDetectorParallel(unsigned int numberOfInstances, std::vector<DetectedObject*>& shapes, DetectionColorItem* settings = NULL);

    ObjectDetectorParallel(unsigned int numberOfInstances, std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage);

    virtual ~ObjectDetectorParallel();

public:
    DetectedObject* findObject();

    void invalidate();

    void initDetectionParams(unsigned int shrink = 1);
        
protected:
    DetectedObject* findBestShape();
    
    void cleanUp();
    
    void generateShapes(DetectedObject* shape, unsigned int size);    

    void generate(unsigned int shapeIndex);
    
    bool findShapeInImagePart(DetectedObject* object);
    
    bool innerShapeFind(DetectedObject* object, unsigned int y, unsigned int x);    
    
    bool rawShapeFind(DetectedObject* object, unsigned int y, unsigned int x);
};

#endif	/* OBJECTDETECTORPARALLEL_H */

