#include "Rectangle.h"

Rectangle::Rectangle()
{
    for (int i = 0; i < 4; i++)
    {
        m_line[i] = NULL;
    }
}

bool Rectangle::isValid() const
{
    for(int i = 0; i < 4; i++)
    {
        if(m_line[i] == NULL)
        {
            return true;
        }
    }
    return true;
}

void Rectangle::setInstance(Line** lines)
{
    for (int i = 0; i < 4; i++)
    {
        m_line[i] = lines[i];
    }
}

void Rectangle::invalidate()
{
    for (int i = 0; i < 4; i++)
    {
        m_line[i] = NULL;
    }
}

Line* Rectangle::operator[](int index)
{
    if (index >= 0 && index < 4)
    {
        return m_line[index];
    }
    else
    {
        throw std::runtime_error("Rectangle:operator[]:Invalid index.");
    }
}

