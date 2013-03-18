/* 
 * File:   CircleDetector.h
 * Author: lubos
 *
 * Created on March 16, 2013, 2:57 PM
 */

#ifndef CIRCLEDETECTOR_H
#define	CIRCLEDETECTOR_H

#include "Circle.h"
#include "ObjectDetector.h"

class CircleDetector : public ObjectDetector
{
protected:            
    std::vector<float> m_angles;
    
    std::vector<Line*> m_ellipses;
    
    Circle* m_bestCircle;
    
    Line* m_tempLine;

public:
    CircleDetector(DetectionColorItem* settings = NULL);

    CircleDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~CircleDetector();

public:
    Circle* findObject();

    void invalidate();

    void initDetectionParams(unsigned int shrink = 1);
        
    void setAngles(std::vector<float> angles);

protected:
    Circle* findBestCircle();

    bool colorMatch(unsigned int failCount = 0);
    
    Line* generateEllipse(int x0, int y0, int radius, float angle);
    
    void generateEllipses(unsigned int ellipseSize);
    
    bool findEllipseInImagePart(unsigned int imagePart, unsigned int ellipseSize);
    
    bool innerEllipseFind(Line* line, unsigned int y, unsigned int x);    
};

#endif	/* CIRCLEDETECTOR_H */

