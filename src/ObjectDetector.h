/* 
 * File:   SquareDetector.h
 * Author: lubos
 *
 * Created on March 18, 2013, 4:00 AM
 */

#ifndef OBJECTDETECTOR_H
#define	OBJECTDETECTOR_H

#include "Shapes/DetectedObject.h"
#include "AbstractObjectDetector.h"


class ObjectDetector : public AbstractObjectDetector
{
protected:                
public:
    ObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings = NULL);

    ObjectDetector(std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage);

    virtual ~ObjectDetector();

public:
    DetectedObject* findObject();

    void invalidate();

    void initDetectionParams(unsigned int shrink = 1);            

protected:
    DetectedObject* findBestShape();
    
    void cleanUp();   
    
    void generateShapes(DetectedObject* object, unsigned int size);        
    
    bool findShapeInImage(DetectedObject* object);
    
    bool innerShapeFind(DetectedObject* object, unsigned int y, unsigned int x);    
    
    bool rawShapeFind(DetectedObject* shape, unsigned int y, unsigned int x, unsigned int ratio, unsigned int base);
};

#endif	/* SQUAREDETECTOR_H */

