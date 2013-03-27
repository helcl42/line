/* 
 * File:   IDetectedObject.h
 * Author: lubos
 *
 * Created on March 26, 2013, 5:22 PM
 */

#ifndef IDETECTEDOBJECT_H
#define	IDETECTEDOBJECT_H

#include "../Line.h"

class IDetectedObject
{
public:
    virtual Vector2<int>* getObjectPoint() = 0;
    
    virtual bool isValid() = 0;
    
    virtual void invalidate() = 0;
    
    virtual Line** getPolygons() = 0;
    
    virtual unsigned int getCountOfPolygons() const = 0;
};

#endif	/* IDETECTEDOBJECT_H */

