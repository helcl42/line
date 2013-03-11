/* 
 * File:   TriangleDetector.h
 * Author: lubos
 *
 * Created on March 10, 2013, 8:30 PM
 */

#ifndef TRIANGLEDETECTOR_H
#define	TRIANGLEDETECTOR_H


#include "Triangle.h"
#include "StraightObjectDetector.h"


class TriangleDetector : public StraightObjectDetector
{
private:
    Triangle* m_bestRectangle;

public:
    TriangleDetector(DetectionColorItem* settings = NULL);

    TriangleDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~TriangleDetector();

    Triangle* detectTriangle();
        
    void invalidate();
    
    Triangle* findBestTriangle();    

protected:
    void initDetectionParams();
};


#endif	/* TRIANGLEDETECTOR_H */

