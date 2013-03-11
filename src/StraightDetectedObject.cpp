#include "StraightDetectedObject.h"


StraightDetectedObejct::StraightDetectedObejct(unsigned int lineCount)
{
    m_lineCount = lineCount;
    m_lines = new Line*[lineCount];
}

StraightDetectedObejct::~StraightDetectedObejct()
{
    SAFE_DELETE_ARRAY(m_lines);
}

void StraightDetectedObejct::setLocked()
{
    setLocked(m_lineCount);
}

void StraightDetectedObejct::setLocked(unsigned int count)
{
    for(unsigned int i = 0; i < count; i++)
    {
        m_lines[i]->locked = true;        
    }
}

Line** StraightDetectedObejct::getLines()
{
    return m_lines;
}

unsigned int StraightDetectedObejct::getLineCount() const
{
    return m_lineCount;
}


Line* StraightDetectedObejct::getAt(unsigned int index) const 
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

void StraightDetectedObejct::setAt(Line* line, unsigned int index)
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

bool StraightDetectedObejct::isValid() const
{
    return isValid(m_lineCount);
}

bool StraightDetectedObejct::isValid(unsigned int count) const
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

void StraightDetectedObejct::invalidate()
{
    for (unsigned int i = 0; i < m_lineCount; i++)
    {
        m_lines[i] = NULL;
    }
}

