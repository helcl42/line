/* 
 * File:   Rectangle.h
 * Author: lubos
 *
 * Created on March 26, 2013, 3:01 AM
 */

#ifndef RECTANGLE_H
#define	RECTANGLE_H

#include "DetectedObject.h"


class Rectangle : public DetectedObject
{
private:
    float m_width;

    float m_height;

public:

    Rectangle()
    : m_width(0), m_height(0) {}

    Rectangle(Polygon<int>* polygon, float width, float height)
    : DetectedObject(polygon), m_width(width), m_height(height) {}

    Rectangle(float width, float height)
    : m_width(width), m_height(height) {}

    virtual ~Rectangle() {}

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

    void generate(float angle = 0)
    {                                                
        double sinAngle = sin(angle * M_PI / 180);
        double cosAngle = cos(angle * M_PI / 180);                
        
        unsigned int w = m_width;
        unsigned int h = m_height;
        
        cleanUp();                       
        
        for (int i = 0; i < h; i++)
        {
            for (int j = 0; j < w; j++)
            {
                if (i == 0 || i == h - 1 || j == 0 || j == w - 1)
                {                                        
                    m_polygon->addPoint(Vector2<int>(j * cosAngle - i * sinAngle, j * sinAngle + i * cosAngle));
                }               
            }
        }      
        
        translateToOrigin();
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

#endif	/* RECTANGLE_H */

