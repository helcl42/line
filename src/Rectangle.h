/* 
 * File:   Rectangle.h
 * Author: lubos
 *
 * Created on February 27, 2013, 5:35 PM
 */

#ifndef RECTANGLE_H
#define	RECTANGLE_H

#include "StraightDetectedObject.h"

class Rectangle : public StraightDetectedObject
{
public:
    Rectangle()
    : StraightDetectedObject(2) {}
    
    virtual ~Rectangle() {}
    
    friend std::ostream& operator<<(std::ostream& out, const Rectangle& linePair);
};

#endif	/* RECTANGLE_H */

