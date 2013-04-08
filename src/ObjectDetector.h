/* 
 * File:   SquareDetector.h
 * Author: lubos
 *
 * Created on March 18, 2013, 4:00 AM
 */

#ifndef SVGOBJECTDETECTOR_H
#define	SVGOBJECTDETECTOR_H

#include "Shapes/DetectedObject.h"
#include "ObjectDetectorThread.h"
#include "AbstractObjectDetector.h"


class SvgObjectDetector : public AbstractObjectDetector
{
protected:                
public:
    SvgObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings = NULL);

    SvgObjectDetector(std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage);

    virtual ~SvgObjectDetector();

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
    
    bool rawShapeFind(DetectedObject* object, unsigned int y, unsigned int x);
};

#endif	/* SQUAREDETECTOR_H */

