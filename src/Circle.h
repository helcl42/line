/* 
 * File:   Circle.h
 * Author: lubos
 *
 * Created on March 16, 2013, 2:44 PM
 */

#ifndef CIRCLE_H
#define	CIRCLE_H

#include <stdexcept>
#include "LineDescribableObject.h"

class Circle : public LineDescribableObject
{    
public:
    Circle() 
    : LineDescribableObject(1) {}
    
    virtual ~Circle() {}            
    
    void clearPoints();
    
    friend std::ostream& operator<<(std::ostream& out, const Circle& circle);
};

#endif	/* CIRCLE_H */

