#include "LinePair.h"

LinePair::LinePair()
{
    m_line[0] = NULL;
    m_line[1] = NULL;
}

bool LinePair::isValid() const
{
    return m_line[0] != NULL && m_line[1] != NULL;
}

Line* LinePair::getFirst() const
{
    return m_line[0];
}

Line* LinePair::getSecond() const
{
    return m_line[1];
}

Line** LinePair::getLines()
{
    return m_line;
}

void LinePair::setLine(Line* l1, Line* l2)
{
    m_line[0] = l1;
    m_line[1] = l2;
}

void LinePair::invalidate()
{
    m_line[0] = NULL;
    m_line[1] = NULL;
}

Line* LinePair::operator[](int index)
{
    if (index >= 0 && index < 2)
    {
        return m_line[index];
    }
    else
    {
        throw std::runtime_error("LinePair:operator[]:Invalid index.");
    }
}
