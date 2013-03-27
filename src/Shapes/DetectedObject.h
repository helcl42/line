/* 
 * File:   DetectedObject.h
 * Author: lubos
 *
 * Created on March 26, 2013, 2:42 AM
 */

#ifndef DETECTEDOBJECT_H
#define	DETECTEDOBJECT_H

#include <cstdlib>
#include "../Utils/Utils.h"
#include "../Pixel/Pixel.h"
#include "../Vector2.h"
#include "../Line.h"
#include "IDetectedObject.h"

class DetectedObject : public IDetectedObject
{
protected:
    Line* m_polygon;

    Vector2<int> m_boundingPointLower;

    Vector2<int> m_boundingPointHigher;

    Pixel<int>* m_color;

public:
    DetectedObject();

    DetectedObject(Line* polygon);

    virtual ~DetectedObject();

public:
    void setPolygon(Line* line);

    Vector2<int>* getLowerBoundingPoint();

    Vector2<int>* getHigherBoundingPoint();

    void rescale(float ratio);
    
    void rescaleToSize(float height);

    void translateToOrigin();

    void rotateByAngle(float angle);

    void viewByAngle(float angle);

    Vector2<int> getOrigin() const;

    unsigned int getWidth() const;

    unsigned int getHeight() const;
    
    unsigned int getCountOfPolygons() const;
    
    Line** getPolygons();
    
    Line* getPolygon();

    void cleanUp();
    
    bool isValid();
    
    void invalidate();

protected:
    void computeBounds();
};

#endif	/* DETECTEDOBJECT_H */

