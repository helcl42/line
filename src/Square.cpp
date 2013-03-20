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
    Vector2<int>* p1 = NULL;
    Vector2<int>* p2 = NULL;
    Vector2<int>* p3 = NULL;
    Line* l1 = getAt(0);
    unsigned int lenFourth = l1->getSize() / 4;

    if (lenFourth > 1)
    {
        p1 = Vector2<int>::getPointBetween(l1->points[0], l1->points[2 * lenFourth]);
        p2 = Vector2<int>::getPointBetween(l1->points[lenFourth], l1->points[3 * lenFourth]);
        p3 = Vector2<int>::getPointBetween(p1, p2);
        SAFE_DELETE(p1);
        SAFE_DELETE(p2);
        return p3;
    }
    return NULL;
}

std::ostream& operator<<(std::ostream& out, const Square& square)
{
    Line* line = *square.getLines();
    out << "<-- Square len = " << line->points.size() << " -->" << std::endl;
    out << *line << std::endl;
    return out;
}
