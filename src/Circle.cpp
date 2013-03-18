#include "Circle.h"

void Circle::clearPoints()
{
    if(m_lines != NULL)
    {
        m_lines[0]->points.clear();
    }
    else
    {
        throw std::runtime_error("Circle == NULL");
    }
}

Vector2<int>* Circle::getObjectPoint()
{
    Line* l1 = m_lines[0];          
    return Vector2<int>::getPointBetween(l1->points[0], l1->points[1]);
}

std::ostream& operator<<(std::ostream& out, const Circle& circle)
{
    Line* line = *circle.getLines();
    out << "<-- Circle len = " << line->points.size() << " -->" << std::endl;
    out << *line << std::endl;
    return out;
}