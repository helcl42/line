#include "LineDescribableObject.h"

LineDescribableObject::LineDescribableObject(unsigned int lineCount)
{
    m_lineCount = lineCount;
    m_lines = new Line*[lineCount];
}

LineDescribableObject::~LineDescribableObject()
{
    SAFE_DELETE_ARRAY(m_lines);
}

void LineDescribableObject::setLocked()
{
    setLocked(m_lineCount);
}

void LineDescribableObject::setLocked(unsigned int count)
{
    for (unsigned int i = 0; i < count; i++)
    {
        m_lines[i]->locked = true;
    }
}

Line** LineDescribableObject::getLines() const
{
    return m_lines;
}

unsigned int LineDescribableObject::getLineCount() const
{
    return m_lineCount;
}

Line* LineDescribableObject::getAt(unsigned int index) const
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

void LineDescribableObject::setAt(Line* line, unsigned int index)
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

bool LineDescribableObject::isValid() const
{
    return isValid(m_lineCount);
}

bool LineDescribableObject::isValid(unsigned int count) const
{
    for (unsigned int i = 0; i < count; i++)
    {
        if (m_lines[i] == NULL)
        {
            return false;
        }
    }
    return true;
}

void LineDescribableObject::invalidate()
{
    for (unsigned int i = 0; i < m_lineCount; i++)
    {
        m_lines[i] = NULL;
    }
}

bool LineDescribableObject::hasLengthInInterval(float divider)
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

