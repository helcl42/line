#include "Circle.h"

void Circle::addPoint(Vector2<int> point)
{
    if (m_lines[0] != NULL)
    {
        m_lines[0]->points.push_back(point);
    }
    else
    {
        throw std::runtime_error("Circle == NULL");
    }
}

void Circle::setLine(Line* line)
{
    m_lines[0] = line;
}

std::ostream& operator<<(std::ostream& out, const Circle& circle)
{
    Line* line = *circle.getLines();
    out << "<-- Circle len = " << line->points.size() << " -->" << std::endl;
    out << *line << std::endl;
    return out;
}