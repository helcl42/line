#include "Rectangle.h"


Vector2<int>* Rectangle::getObjectPoint()
{
    Line* l1 = m_lines[0];
    Line* l2 = m_lines[2];
    unsigned int halfLength;

    if (l1->points.size() < l2->points.size())
    {
        halfLength = l1->points.size() / 2;
    }
    else
    {
        halfLength = l2->points.size() / 2;
    }

    return Vector2<int>::getPointBetween(l1->points[halfLength], l2->points[halfLength]);
}

std::ostream& operator<<(std::ostream& out, const Rectangle& linePair)
{
    Line** lines = linePair.getLines();
    out << "<-- Rectangle -->" << std::endl;
    
    for(unsigned int i = 0; i < linePair.getLineCount(); i++)
    {
        out << *lines[i] << std::endl;
    }
    return out;
}
