/* 
 * File:   RectangleDetector.h
 * Author: lubos
 *
 * Created on March 7, 2013, 3:22 AM
 */

#ifndef RECTANGLEDETECTOR_H
#define	RECTANGLEDETECTOR_H

#include "Rectangle.h"
#include "StraightObjectDetector.h"


class RectangleDetector : public StraightObjectDetector
{
private:
    Rectangle* m_bestRectangle;

public:
    RectangleDetector(DetectionColorItem* settings = NULL);

    RectangleDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~RectangleDetector();

public:    
    StraightDetectedObject* findObject();
        
    void invalidate();           
    
    void initDetectionParams(unsigned int shrink = 1);

protected:
    StraightDetectedObject* findBestRectangle();     
    
    bool colorMatch(unsigned int failCount = 0);
};

#endif	/* RECTANGLEDETECTOR_H */

