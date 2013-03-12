/* 
 * File:   Triangle.h
 * Author: lubos
 *
 * Created on March 10, 2013, 8:32 PM
 */

#ifndef TRIANGLE_H
#define	TRIANGLE_H

#include "StraightDetectedObject.h"


class Triangle : public StraightDetectedObject
{ 
public:
    Triangle()
     : StraightDetectedObject(2) {}

    virtual ~Triangle() {}    
    
    friend std::ostream& operator<<(std::ostream& out, const Triangle& linePair);
};

#endif	/* TRIANGLE_H */

