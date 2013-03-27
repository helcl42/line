/* 
 * File:   GeneralObject.h
 * Author: lubos
 *
 * Created on March 26, 2013, 2:46 AM
 */

#ifndef GENERALOBJECT_H
#define	GENERALOBJECT_H

#include "DetectedObject.h"


class GeneralObject : public DetectedObject
{
public:
    GeneralObject() {}
    
    GeneralObject(Line* polygon) 
        : DetectedObject(polygon) {}
    
    Vector2<int>* getObjectPoint()
    {
        unsigned int sizeQuad = m_polygon->getSize() / 4;
        
        Vector2<int>* a = Vector2<int>::getPointBetween(m_polygon->getPoint(0), m_polygon->getPoint(sizeQuad * 2));
        Vector2<int>* b = Vector2<int>::getPointBetween(m_polygon->getPoint(sizeQuad), m_polygon->getPoint(sizeQuad * 3));
        return Vector2<int>::getPointBetween(a, b);
    }        
};

#endif	/* GENERALOBJECT_H */

