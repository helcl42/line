#include "LinePair.h"

LinePair::LinePair()
{
    m_lineCount = 2;
    m_lines = new Line*[2];
}

LinePair::~LinePair()
{
    SAFE_DELETE_ARRAY(m_lines);
}

void LinePair::setLocked()
{
     for (unsigned int i = 0; i < m_lineCount; i++)
    {
        m_lines[i]->locked = true;
    }
}

Line** LinePair::getPolygons()
{
    return m_lines;
}

unsigned int LinePair::getCountOfPolygons() const
{
    return m_lineCount;
}

Line* LinePair::getAt(unsigned int index) const
{
    if (index >= 0 && index < m_lineCount)
    {
        return m_lines[index];
    }
    else
    {
        throw std::runtime_error("StraightDetectedObject:getAt:Invalid index.");
    }
}

void LinePair::setAt(Line* line, unsigned int index)
{
    if (index >= 0 && index < m_lineCount)
    {
        m_lines[index] = line;
    }
    else
    {
        throw std::runtime_error("StraightDetectedObject:setAt:Invalid index.");
    }
}

bool LinePair::isValid()
{
    for (unsigned int i = 0; i < m_lineCount; i++)
    {
        if (m_lines[i] == NULL)
        {
            return false;
        }
    }
    return true;
}

void LinePair::invalidate()
{
    for (unsigned int i = 0; i < m_lineCount; i++)
    {
        m_lines[i] = NULL;
    }
}

bool LinePair::hasLengthInInterval(float divider)
{
    for (unsigned int i = 0; i < m_lineCount; i++)
    {
        if (m_lines[i]->length + DetectionParams::minLineLengthTreshold / divider < m_lines[i]->length
                || m_lines[i]->length - DetectionParams::minLineLengthTreshold / divider > m_lines[i]->length)
        {
            return false;
        }

    }
    return true;
}

Vector2<int>* LinePair::getObjectPoint()
{
    Line* l1 = getAt(0);
    Line* l2 = getAt(1);
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


std::ostream& operator<<(std::ostream& out, LinePair& linePair)
{
    Line** lines = linePair.getPolygons();
    out << "<-- LinePair -->" << std::endl;
    
    for(unsigned int i = 0; i < linePair.getCountOfPolygons(); i++)
    {
        out << *lines[i] << std::endl;
    }
    return out;
}



