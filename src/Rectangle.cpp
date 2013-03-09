#include "Rectangle.h"

Rectangle::Rectangle()
{
    for (int i = 0; i < 4; i++)
    {
        m_line[i] = NULL;
    }
}

bool Rectangle::isValid(unsigned int a) const
{
    for(unsigned int i = 0; i < a; i++)
    {
        if(m_line[i] == NULL)
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
        m_line[i]->locked = true;        
    }
}

void Rectangle::setInstance(Line** lines)
{
    for (unsigned int i = 0; i < 4; i++)
    {
        m_line[i] = lines[i];
    }
}

void Rectangle::invalidate()
{
    for (unsigned int i = 0; i < 4; i++)
    {
        m_line[i] = NULL;
    }
}

Line* Rectangle::getAt(unsigned int index) const 
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

void Rectangle::setAt(Line* line, unsigned int index)
{
    if (index >= 0 && index < 4)
    {
        m_line[index] = line;
    }
    else
    {
        throw std::runtime_error("Rectangle:operator[]:Invalid index.");
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

