/* 
 * File:   Triangle.h
 * Author: lubos
 *
 * Created on March 10, 2013, 8:32 PM
 */

#ifndef TRIANGLE_H
#define	TRIANGLE_H

#include "LineDescribableObject.h"

class Triangle : public LineDescribableObject
{    
public:
    Triangle();
    
    virtual ~Triangle();
    
    void rotateByAngle(double angle);
    
    void projectByAngle(double angle);
    
    void clearPoints();
    
    void addPoint(Vector2<int> point);
    
    Vector2<int>* getObjectPoint();
    
    friend std::ostream& operator<<(std::ostream& out, const Triangle& circle);
};

#endif	/* TRIANGLE_H */

