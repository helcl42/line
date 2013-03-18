#include "Square.h"

Square::Square()
: LineDescribableObject(1)
{
    m_lines[0] = new Line();
}

Square::~Square()
{
    SAFE_DELETE(m_lines[0]);
}

void Square::clearPoints()
{
    Line* line = getAt(0);
    if (line != NULL)
    {
        line->points.clear();
    }
    else
    {
        throw std::runtime_error("Square == NULL");
    }
}

void Square::addPoint(Vector2<int> point)
{
    Line* line = getAt(0);    
    line->points.push_back(point);
}

void Square::projectByAngle(double angle)
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

void Square::rotateByAngle(double angle)
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

Vector2<int>* Square::getObjectPoint()
{
    Line* l1 = getAt(0);

    if (l1->points.size() > 1)
    {
        return Vector2<int>::getPointBetween(l1->points[0], l1->points[1]);
    }
    return NULL;
}

std::ostream& operator<<(std::ostream& out, const Square & circle)
{
    Line* line = *circle.getLines();
    out << "<-- Square len = " << line->points.size() << " -->" << std::endl;
    out << *line << std::endl;
    return out;
}
