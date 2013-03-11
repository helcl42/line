/* 
 * File:   Rectangle.h
 * Author: lubos
 *
 * Created on February 27, 2013, 5:35 PM
 */

#ifndef RECTANGLE_H
#define	RECTANGLE_H

#include "StraightDetectedObject.h"

class Rectangle : public StraightDetectedObejct
{
public:
    Rectangle()
    : StraightDetectedObejct(4) {}
    
    virtual ~Rectangle() {}
};

#endif	/* RECTANGLE_H */

