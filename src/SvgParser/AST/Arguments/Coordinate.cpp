#include "Coordinate.h"

float Coordinate::getValue() const
{
    return m_value;
}

void Coordinate::setValue(float value)
{
    this->m_value = value;
}

void Coordinate::draw(Line* polygon)
{
}

void Coordinate::print()
{
    std::cout << "Coordinate = " << m_value << std::endl;
}
