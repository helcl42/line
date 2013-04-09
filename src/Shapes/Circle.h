/* 
 * File:   Circle.h
 * Author: lubos
 *
 * Created on March 26, 2013, 3:00 AM
 */

#ifndef ICIRCLE_H
#define	ICIRCLE_H

#include "../Utils/Utils.h"
#include "DetectedObject.h"

class Circle : public DetectedObject
{
private:
    float m_radius;

public:

    Circle()
    : m_radius(0) {}

    Circle(Line<int>* polygon, float radius)
    : DetectedObject(polygon), m_radius(radius) {}

    Circle(float radius)
    : m_radius(radius) {}

    virtual ~Circle() {}

public:

    Vector2<int>* getObjectPoint()
    {
        unsigned int sizeQuad = m_polygon->getSize() / 4;

        Vector2<int>* a = Vector2<int>::getPointBetween(m_polygon->getPoint(0), m_polygon->getPoint(sizeQuad * 2));
        Vector2<int>* b = Vector2<int>::getPointBetween(m_polygon->getPoint(sizeQuad), m_polygon->getPoint(sizeQuad * 3));
        Vector2<int>* c = Vector2<int>::getPointBetween(a, b);
        SAFE_DELETE(a);
        SAFE_DELETE(b);
        return c;
    }

    void generate()
    {        
        cleanUp();
        
        //x0 = y0 == radius
        int f = 1 - m_radius;
        int deltaX = 1;
        int deltaY = -2 * m_radius;
        int x = 0;
        int y = m_radius;        

        m_polygon->addPoint(m_radius * 2, m_radius);
        m_polygon->addPoint(0, m_radius);
        m_polygon->addPoint(m_radius, m_radius * 2);
        m_polygon->addPoint(m_radius, 0);

        while (x < y)
        {
            if (f >= 0)
            {
                y--;
                deltaY += 2;
                f += deltaY;
            }

            x++;
            deltaX += 2;
            f += deltaX;

            m_polygon->addPoint(m_radius + y, m_radius + x);
            m_polygon->addPoint(m_radius + y, m_radius - x);
            m_polygon->addPoint(m_radius - y, m_radius + x);
            m_polygon->addPoint(m_radius - y, m_radius - x);

            m_polygon->addPoint(m_radius + x, m_radius + y);
            m_polygon->addPoint(m_radius + x, m_radius - y);
            m_polygon->addPoint(m_radius - x, m_radius + y);
            m_polygon->addPoint(m_radius - x, m_radius - y);
        }        
    }

    float getRadius() const
    {
        return m_radius;
    }

    void setRadius(float radius)
    {
        this->m_radius = radius;
    }
};

#endif	/* CIRCLE_H */

