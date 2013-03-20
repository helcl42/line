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

class TriangleDetector : public ObjectDetector
{
protected:            
    std::vector<float> m_angles;
    
    std::vector<Triangle*> m_triangles;
    
    Triangle* m_bestTriangle;
    
    Line* m_tempLine;

public:
    TriangleDetector(DetectionColorItem* settings = NULL);

    TriangleDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~TriangleDetector();

public:
    Triangle* findObject();

    void invalidate();

    void initDetectionParams(unsigned int shrink = 1);
        
    void setAngles(std::vector<float> angles);

protected:
    Triangle* findBestTriangle();

    bool colorMatch(unsigned int failCount = 0);
    
    Triangle* generateTriangle(int x0, int y0, int size, float angle);
    
    void generateTriangles(unsigned int triangleSize);
    
    bool findTriangleInImagePart(unsigned int imagePart, unsigned int ellipseSize);
    
    bool innerTriangleFind(Triangle* line, unsigned int y, unsigned int x);    
    
    bool rawTriangleFind(Triangle* square, unsigned int y, unsigned int x);
};

#endif	/* TRIANGLEDETECTOR_H */

