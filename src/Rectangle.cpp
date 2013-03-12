#include "Rectangle.h"

std::ostream& operator<<(std::ostream& out, const Rectangle& linePair)
{
    Line** lines = linePair.getLines();
    out << "<-- Rectangle -->" << std::endl;
    
    for(unsigned int i = 0; i < linePair.getLineCount(); i++)
    {
        out << *lines[i] << std::endl;
    }
    return out;
}
