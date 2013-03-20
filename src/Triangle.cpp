#include "Triangle.h"

Triangle::Triangle()
: LineDescribableObject(1)
{
    m_lines[0] = new Line();
}

Triangle::~Triangle()
{
    SAFE_DELETE(m_lines[0]);
}

void Triangle::clearPoints()
{
    Line* line = getAt(0);
    if (line != NULL)
    {
        line->points.clear();
    }
    else
    {
        throw std::runtime_error("Triangle == NULL");
    }
}

void Triangle::addPoint(Vector2<int> point)
{
    Line* line = getAt(0);    
    line->points.push_back(point);
}

void Triangle::projectByAngle(double angle)
{
    Line* line = getAt(0);
    double sinAngle = sin(angle * M_PI / 180); 
    
    for (unsigned int i = 0; i < line->getSize(); i++)
    {
        Vector2<int> point = line->points[i];        
        line->points[i].x = point.x;
        line->points[i].y = point.y * sinAngle;
    }               
}

void Triangle::rotateByAngle(double angle)
{
    Line* line = getAt(0);
    double sinAngle = sin(angle * M_PI / 180);
    double cosAngle = cos(angle * M_PI / 180);
    
    for (unsigned int i = 0; i < line->getSize(); i++)
    {
        Vector2<int> point = line->points[i];
        line->points[i] = Vector2<int>(point.x * sinAngle + point.y * cosAngle, point.x * cosAngle - point.y * sinAngle);        
    }
}

Vector2<int>* Triangle::getObjectPoint()
{
    Vector2<int>* p1 = NULL;
    Vector2<int>* p2 = NULL;
    Vector2<int>* p3 = NULL;
    Line* l1 = getAt(0);    
    unsigned int lenSixth = l1->getSize() / 6;

    if (lenSixth > 1)
    {
        p1 = Vector2<int>::getPointBetween(l1->points[lenSixth], l1->points[3 * lenSixth]);
        p2 = Vector2<int>::getPointBetween(l1->points[lenSixth], l1->points[5 * lenSixth]);
        p3 = Vector2<int>::getPointBetween(p1, p2);
        SAFE_DELETE(p1);
        SAFE_DELETE(p2);
        return p3; 
    }    
    return NULL;
}

std::ostream& operator<<(std::ostream& out, const Triangle& triangle)
{
    Line* line = *triangle.getLines();
    out << "<-- Triangle len = " << line->points.size() << " -->" << std::endl;
    out << *line << std::endl;
    return out;
}
