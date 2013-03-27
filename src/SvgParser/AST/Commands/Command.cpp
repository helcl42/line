#include "Command.h"

std::vector<Argument*> Command::getArgs() const
{
    return m_args;
}

void Command::setArgs(std::vector<Argument*> args)
{
    this->m_args = args;
}

bool Command::isRelative() const
{
    return m_relative;
}

void Command::setRelative(bool relative)
{
    this->m_relative = relative;
}
