#include "StraightDetectedObject.h"


StraightDetectedObject::StraightDetectedObject(unsigned int lineCount)
{
    m_lineCount = lineCount;
    m_lines = new Line*[lineCount];
}

StraightDetectedObject::~StraightDetectedObject()
{
    SAFE_DELETE_ARRAY(m_lines);
}

void StraightDetectedObject::setLocked()
{
    setLocked(m_lineCount);
}

void StraightDetectedObject::setLocked(unsigned int count)
{
    for(unsigned int i = 0; i < count; i++)
    {
        m_lines[i]->locked = true;        
    }
}

Line** StraightDetectedObject::getLines()
{
    return m_lines;
}

unsigned int StraightDetectedObject::getLineCount() const
{
    return m_lineCount;
}


Line* StraightDetectedObject::getAt(unsigned int index) const 
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

void StraightDetectedObject::setAt(Line* line, unsigned int index)
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

bool StraightDetectedObject::isValid() const
{
    return isValid(m_lineCount);
}

bool StraightDetectedObject::isValid(unsigned int count) const
{
    for(unsigned int i = 0; i < count; i++)
    {
        if(m_lines[i] == NULL)
        {
            return false;
        }
    }
    return true;
}

void StraightDetectedObject::invalidate()
{
    for (unsigned int i = 0; i < m_lineCount; i++)
    {
        m_lines[i] = NULL;
    }
}

