#include "PositionFactory.h"

std::vector<Vector2<float> > PositionFactory::m_positions;

std::vector<Vector2<float> > PositionFactory::m_basePoints;

unsigned int PositionFactory::m_positionsCounter = 0;

void PositionFactory::addPosition(Vector2<int> coordiantePair)
{
    m_positions.push_back(Vector2<float>(coordiantePair.x, coordiantePair.y));    
    m_positionsCounter++;
}

void PositionFactory::addPosition(Vector2<float> coordiantePair)
{
    m_positions.push_back(coordiantePair);    
    m_positionsCounter++;
}

void PositionFactory::addPosition(CoordinatePair* pair)
{
    m_positions.push_back(Vector2<float>(pair->GetNumberX()->getValue(), pair->GetNumberY()->getValue()));    
    m_positionsCounter++;
}

Vector2<float> PositionFactory::getFirstPosition()
{
    if (m_positions.size() >= m_positionsCounter && m_positionsCounter > 0)
    {
        return m_positions.front();
    }
    else
    {
        return m_basePoints.back();
    }
}

Vector2<float> PositionFactory::getLastPosition()
{
    if (m_positions.size() >= m_positionsCounter && m_positionsCounter > 0)
    {
        return m_positions.back();
    }
    else
    {        
        return m_basePoints.back();
    }
}

Vector2<float> PositionFactory::getPositionAt(int index)
{
    if (index >= 0 && index < m_positions.size())
    {
        return m_positions[index];
    }
    else
    {
        throw std::runtime_error("Positiion factory: Index is out of bounds");
    }
}

void PositionFactory::addBasePoint(Vector2<int> coordiantePair)
{
    m_basePoints.push_back(Vector2<float>(coordiantePair.x, coordiantePair.y));
}

void PositionFactory::addBasePoint(Vector2<float> coordiantePair)
{
    m_basePoints.push_back(coordiantePair);
}

void PositionFactory::addBasePoint(CoordinatePair * pair)
{
    m_basePoints.push_back(Vector2<float>(pair->GetNumberX()->getValue(), pair->GetNumberY()->getValue()));
}

void PositionFactory::addBasePoint(float x, float y)
{
    m_basePoints.push_back(Vector2<float>(x, y));
}

Vector2<float> PositionFactory::getLastBasePointPosition()
{
    if (m_basePoints.size() > 0)
    {
        return m_basePoints.back();
    }
    else
    {
        return Vector2<float>(0, 0);
    }
}

Vector2<float> PositionFactory::getFirstBasePointPosition()
{
    return m_basePoints.front();
}

Vector2<float> PositionFactory::getBasePointPositionAt(int index)
{
    if (index >= 0 && index < m_positions.size())
    {
        return m_basePoints[index];
    }
    else
    {
        throw std::runtime_error("Positiion factory: Index is out of bounds");
    }
}

