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
    Triangle()
     : LineDescribableObject(2) {}

    virtual ~Triangle() {}    
    
    Vector2<int>* getObjectPoint();
    
    friend std::ostream& operator<<(std::ostream& out, const Triangle& linePair);
};

#endif	/* TRIANGLE_H */

