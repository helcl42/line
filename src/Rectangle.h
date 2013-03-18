/* 
 * File:   Rectangle.h
 * Author: lubos
 *
 * Created on February 27, 2013, 5:35 PM
 */

#ifndef RECTANGLE_H
#define	RECTANGLE_H

#include "LineDescribableObject.h"

class Rectangle : public LineDescribableObject
{
public:
    Rectangle()
    : LineDescribableObject(2) {}
    
    virtual ~Rectangle() {}
    
    Vector2<int>* getObjectPoint();
    
    friend std::ostream& operator<<(std::ostream& out, const Rectangle& linePair);
};

#endif	/* RECTANGLE_H */

