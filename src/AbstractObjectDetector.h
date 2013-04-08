/* 
 * File:   AbstractObjectDetector.h
 * Author: lubos
 *
 * Created on April 9, 2013, 12:29 AM
 */

#ifndef ABSTRACTOBJECTDETECTOR_H
#define	ABSTRACTOBJECTDETECTOR_H

#include "AbstractDetector.h"
#include "Shapes/GeneralObject.h"
#include "Shapes/DetectedObject.h"
#include "ImageFilters/ImageFilterFactory.h"

class AbstractObjectDetector : public AbstractDetector
{
protected:            
    std::vector<float> m_angles;
    
    std::vector<DetectedObject*> m_shapes;
    
    std::vector<DetectedObject*> m_detectedShapes;
    
    DetectedObject* m_bestMatch;
    
    unsigned int m_shrink;
    
public:
    AbstractObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings = NULL);

    AbstractObjectDetector(std::vector<DetectedObject*>& shapes, ImageMap<float>* image, Image<float>* colorImage);

    virtual ~AbstractObjectDetector();        

public:   
    void setAngles(std::vector<float> angles);    
    
    void setShrink(unsigned int shrink);
    
private:    
    bool colorMatch(unsigned int failCount = 0);
};


#endif	/* ABSTRACTOBJECTDETECTOR_H */

