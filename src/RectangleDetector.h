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

    Rectangle* detectRectangle();
        
    void invalidate();
    
    Rectangle* findBestRectangle();    

protected:
    void initDetectionParams();
};

#endif	/* RECTANGLEDETECTOR_H */

