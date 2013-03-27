/* 
 * File:   Square.h
 * Author: lubos
 *
 * Created on March 18, 2013, 3:09 AM
 */

#ifndef SQUARE_H
#define	SQUARE_H

#include "Shapes/LinePair.h"

class Square : public LinePair
{    
public:
    Square();
    
    virtual ~Square();
    
    void rotateByAngle(double angle);
    
    void projectByAngle(double angle);
    
    void clearPoints();
    
    void addPoint(Vector2<int> point);
    
    Vector2<int>* getObjectPoint();
    
    friend std::ostream& operator<<(std::ostream& out, const Square& circle);
};

#endif	/* SQUARE_H */

