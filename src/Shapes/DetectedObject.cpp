#include "DetectedObject.h"

DetectedObject::DetectedObject()
: m_polygon(NULL)
{
    m_color = new PixelRGB<int>();
}

DetectedObject::DetectedObject(Polygon<int>* polygon)
: m_polygon(polygon)
{
    computeBounds();
    translateToOrigin();
    m_color = new PixelRGB<int>();
}

DetectedObject::~DetectedObject()
{
    SAFE_DELETE(m_polygon);
    SAFE_DELETE(m_color)
}

void DetectedObject::setPolygon(Polygon<int>* line)
{
    m_polygon = line;
    computeBounds();
    translateToOrigin();
}

Vector2<int>* DetectedObject::getLowerBoundingPoint()
{
    return &m_boundingPointLower;
}

Vector2<int>* DetectedObject::getHigherBoundingPoint()
{
    return &m_boundingPointHigher;
}

void DetectedObject::createBatch(float size, float angle)
{
    rescaleToSize(size, false);
    rotateByAngle(angle, true);
}

void DetectedObject::rescale(float ratio, bool shift)
{
    for (unsigned int i = 0; i < m_polygon->getSize(); i++)
    {
        Vector2<int> point = m_polygon->getPoint(i);
        m_polygon->setPoint(point.x * ratio, point.y * ratio, i);
    }

    if (shift)
    {
        computeBounds();
        translateToOrigin();
    }
}

void DetectedObject::rescaleToSize(float size, bool shift)
{
    computeBounds();

    unsigned int width = getWidth();
    unsigned int height = getHeight();
    unsigned int max;

    if (width > height)
    {
        max = width;
    }
    else
    {
        max = height;
    }

    rescale(size / max, shift);
}

void DetectedObject::translateToOrigin()
{
    Vector2<int> translation;
    Vector2<int>* point;

    translation.x = m_boundingPointLower.x < 0 ? -m_boundingPointLower.x : -m_boundingPointLower.x;
    translation.y = m_boundingPointLower.y < 0 ? -m_boundingPointLower.y : -m_boundingPointLower.y;

    if (m_boundingPointLower.x == 0) translation.x = 0;
    if (m_boundingPointLower.y == 0) translation.y = 0;

    for (unsigned int i = 0; i < m_polygon->getSize(); i++)
    {
        point = m_polygon->getPointPtr(i);
        m_polygon->setPoint(point->x + translation.x, point->y + translation.y, i);
    }

    computeBounds();
}

void DetectedObject::rotateByAngle(float angle, bool shift)
{
    Vector2<int>* tempVec;
    double sinAngle, cosAngle;   
    
    while (angle > 0)
    {
        if(angle > 90)
        {
            sinAngle = 1;
            cosAngle = 0;
        }
        else
        {
            sinAngle = sin(angle * M_PI / 180);
            cosAngle = cos(angle * M_PI / 180);
        }
            
        std::cout << "AnGLE = " << angle << std::endl;
        for (int i = 0; i < m_polygon->getSize(); i++)
        {
            tempVec = m_polygon->getPointPtr(i);
            m_polygon->setPoint(tempVec->x * cosAngle - tempVec->y * sinAngle, tempVec->x * sinAngle + tempVec->y * cosAngle, i);
        }
        angle -= 90;
    }

    if (shift)
    {
        computeBounds();
        translateToOrigin();
    }
}

void DetectedObject::viewByAngle(float angle, bool shift)
{
    Vector2<int>* tempVec;
    double sinAngle = sin(angle * M_PI / 180);

    for (int i = 0; i < m_polygon->getSize(); i++)
    {
        tempVec = m_polygon->getPointPtr(i);
        m_polygon->setPoint(tempVec->x, tempVec->y * sinAngle, i);
    }

    if (shift)
    {
        computeBounds();
        translateToOrigin();
    }
}

Vector2<int> DetectedObject::getOrigin() const
{
    int originX = (m_boundingPointHigher.x - m_boundingPointLower.x) / 2;
    int originY = (m_boundingPointHigher.y - m_boundingPointLower.y) / 2;

    return Vector2<int>(originX, originY);
}

unsigned int DetectedObject::getWidth() const
{
    return m_boundingPointHigher.x - m_boundingPointLower.x;
}

unsigned int DetectedObject::getHeight() const
{
    return m_boundingPointHigher.y - m_boundingPointLower.y;
}

bool DetectedObject::isValid()
{
    if (m_polygon != NULL)
    {
        return true;
    }
    return false;
}

void DetectedObject::cleanUp()
{
    if (m_polygon != NULL)
    {
        m_polygon->deletePoints();
    }
    else
    {
        m_polygon = new Polygon<int>();
    }
}

Polygon<int>* DetectedObject::getPolygon()
{
    return m_polygon;
}

Polygon<int>** DetectedObject::getPolygons()
{
    return &m_polygon;
}

void DetectedObject::addPoint(int x, int y)
{
    m_polygon->addPoint(x, y);
}

unsigned int DetectedObject::getCountOfPolygons() const
{
    return 1;
}

void DetectedObject::invalidate()
{
    if (m_polygon != NULL)
    {
        SAFE_DELETE(m_polygon);
    }
}

void DetectedObject::computeBounds()
{
    int minX = m_polygon->getBegin().x;
    int minY = m_polygon->getBegin().y;
    int maxX = m_polygon->getEnd().x;
    int maxY = m_polygon->getEnd().y;

    if (m_polygon != NULL)
    {
        for (unsigned int i = 0; i < m_polygon->getSize(); i++)
        {
            Vector2<int> point = m_polygon->getPoint(i);
            if (minX > point.x)
            {
                minX = point.x;
            }

            if (minY > point.y)
            {
                minY = point.y;
            }

            if (maxX < point.x)
            {
                maxX = point.x;
            }

            if (maxY < point.y)
            {
                maxY = point.y;
            }
        }

        m_boundingPointLower = Vector2<int>(minX, minY);
        m_boundingPointHigher = Vector2<int>(maxX, maxY);
    }
}

