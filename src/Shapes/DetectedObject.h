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
    Line<int>* m_polygon;

    Vector2<int> m_boundingPointLower;

    Vector2<int> m_boundingPointHigher;

    Pixel<int>* m_color;

public:
    DetectedObject();

    DetectedObject(Line<int>* polygon);

    virtual ~DetectedObject();

public:
    void setPolygon(Line<int>* line);

    Vector2<int>* getLowerBoundingPoint();

    Vector2<int>* getHigherBoundingPoint();

    void rescale(float ratio, bool shift = true);
    
    void rescaleToSize(float height, bool shift = true);
    
    void createBatch(float size, float angle, bool direction);

    void translateToOrigin();

    void rotateByAngle(float angle, bool direction, bool shift = true);

    void viewByAngle(float angle, bool shift = true);

    Vector2<int> getOrigin() const;

    unsigned int getWidth() const;

    unsigned int getHeight() const;
    
    unsigned int getCountOfPolygons() const;
    
    Line<int>** getPolygons();
    
    Line<int>* getPolygon();

    void cleanUp();
    
    void addPoint(int x, int y);
    
    bool isValid();
    
    void invalidate();

protected:
    void computeBounds();
};

#endif	/* DETECTEDOBJECT_H */

