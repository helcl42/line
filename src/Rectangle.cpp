#include "Rectangle.h"

Rectangle::Rectangle()
{
    for (int i = 0; i < 4; i++)
    {
        m_lines[i] = NULL;
    }
}

bool Rectangle::isValid(unsigned int a) const
{
    for(unsigned int i = 0; i < a; i++)
    {
        if(m_lines[i] == NULL)
        {
            return false;
        }
    }
    return true;
}

void Rectangle::setLocked(unsigned int a)
{
    for(unsigned int i = 0; i < a; i++)
    {
        m_lines[i]->locked = true;        
    }
}

void Rectangle::setInstance(Line** lines)
{
    for (unsigned int i = 0; i < 4; i++)
    {
        m_lines[i] = lines[i];
    }
}

void Rectangle::invalidate()
{
    for (unsigned int i = 0; i < 4; i++)
    {
        m_lines[i] = NULL;
    }
}

Line* Rectangle::getAt(unsigned int index) const 
{
    if (index >= 0 && index < 4)
    {
        return m_lines[index];
    }
    else
    {
        throw std::runtime_error("Rectangle:getAt:Invalid index.");
    }
}

void Rectangle::setAt(Line* line, unsigned int index)
{
    if (index >= 0 && index < 4)
    {
        m_lines[index] = line;
    }
    else
    {
        throw std::runtime_error("Rectangle:setAt:Invalid index.");
    }
}

Line** Rectangle::getLines()
{
    return m_lines;
}

Line* Rectangle::operator[](int index)
{
    if (index >= 0 && index < 4)
    {
        return m_lines[index];
    }
    else
    {
        throw std::runtime_error("Rectangle:operator[]:Invalid index.");
    }
}

