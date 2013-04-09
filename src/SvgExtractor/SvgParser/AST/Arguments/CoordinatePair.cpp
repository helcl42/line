#include "CoordinatePair.h"

Coordinate* CoordinatePair::GetNumberX() const
{
    return m_numberX;
}

void CoordinatePair::SetNumberX(Coordinate* numberX)
{
    this->m_numberX = numberX;
}

Coordinate* CoordinatePair::GetNumberY() const
{
    return m_numberY;
}

void CoordinatePair::SetNumberY(Coordinate* numberY)
{
    this->m_numberY = numberY;
}

void CoordinatePair::draw(Line<int>* polygon)
{
}

void CoordinatePair::print()
{
    std::cout << "CoordinatePair: " << "(" << m_numberX->getValue() << ", " << m_numberY->getValue() << ")" << std::endl;
}
