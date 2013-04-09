/* 
 * File:   Ellipse.h
 * Author: lubos
 *
 * Created on March 26, 2013, 3:56 AM
 */

#ifndef ELLIPSE_H
#define	ELLIPSE_H

#include "DetectedObject.h"

class Ellipse : public DetectedObject
{
private:
    float m_width;

    float m_height;

public:

    Ellipse()
    : m_width(0), m_height(0)
    {
    }

    Ellipse(Line<int>* polygon, float width, float height)
    : DetectedObject(polygon), m_width(width), m_height(height)
    {
    }

    Ellipse(float width, float height)
    : m_width(width), m_height(height)
    {
    }

    virtual ~Ellipse()
    {
    }

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
        int f = 1 - m_width;
        int deltaX = 1;
        int deltaY = -2 * m_height;
        int x = 0;
        int y = m_height;
        float ellipse = m_height > m_width ? m_width / m_height : m_height / m_width;

        cleanUp();
        
        m_polygon->addPoint(m_height, 0);
        m_polygon->addPoint(-m_height, 0);
        m_polygon->addPoint(0, m_width * ellipse);
        m_polygon->addPoint(0, -m_width * ellipse);

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

            m_polygon->addPoint(y, x * ellipse);
            m_polygon->addPoint(y, -x * ellipse);
            m_polygon->addPoint(-y, x * ellipse);
            m_polygon->addPoint(-y, -x * ellipse);

            m_polygon->addPoint(x, y * ellipse);
            m_polygon->addPoint(x, -y * ellipse);
            m_polygon->addPoint(-x, y * ellipse);
            m_polygon->addPoint(-x, -y * ellipse);
        }        
    }

    float getHeight() const
    {
        return m_height;
    }

    void setHeight(float height)
    {
        this->m_height = height;
    }

    float getWidth() const
    {
        return m_width;
    }

    void setWidth(float width)
    {
        this->m_width = width;
    }
};

#endif	/* ELLIPSE_H */

