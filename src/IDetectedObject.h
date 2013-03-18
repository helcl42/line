/* 
 * File:   DetectedObject.h
 * Author: lubos
 *
 * Created on March 10, 2013, 8:34 PM
 */

#ifndef DETECTEDOBJECT_H
#define	DETECTEDOBJECT_H

#include "Line.h"

class IDetectedObject
{
public:
    virtual bool isValid(unsigned int a) const = 0;
    
    virtual void setLocked(unsigned int a) = 0;
    
    virtual void invalidate() = 0;

    virtual Vector2<int>* getObjectPoint() = 0;
};

#endif	/* DETECTEDOBJECT_H */

