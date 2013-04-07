/* 
 * File:   SquareDetector.h
 * Author: lubos
 *
 * Created on March 18, 2013, 4:00 AM
 */

#ifndef SVGOBJECTDETECTOR_H
#define	SVGOBJECTDETECTOR_H

#include "ObjectDetector.h"
#include "Shapes/DetectedObject.h"


class SvgObjectDetector : public ObjectDetector
{
protected:            
    std::vector<float> m_angles;
    
    std::vector<DetectedObject*>& m_shapes;
    
    std::vector<DetectedObject*> m_detectedShapes;
    
    DetectedObject* m_bestMatch;

    unsigned int m_shrink;

public:
    SvgObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings = NULL);

    SvgObjectDetector(std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage);

    virtual ~SvgObjectDetector();

public:
    DetectedObject* findObject();

    void invalidate();

    void initDetectionParams(unsigned int shrink = 1);
        
    void setAngles(std::vector<float> angles);

protected:
    DetectedObject* findBestShape();
    
    void setShrink(unsigned int shrink);
    
    void cleanUp();

    bool colorMatch(unsigned int failCount = 0);
    
    void generateShapes(unsigned int shapeIndex, unsigned int size);        
    
    bool findShapeInImagePart(DetectedObject* object);
    
    bool innerShapeFind(DetectedObject* object, unsigned int y, unsigned int x);    
    
    bool rawShapeFind(DetectedObject* object, unsigned int y, unsigned int x);
};

#endif	/* SQUAREDETECTOR_H */

