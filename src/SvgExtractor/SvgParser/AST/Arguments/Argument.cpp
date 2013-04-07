#include "Argument.h"

ArgumentType Argument::GetArgumentType() const
{
    return m_argumentType;
}

void Argument::SetArgumentType(ArgumentType argumentType)
{
    this->m_argumentType = argumentType;
} 