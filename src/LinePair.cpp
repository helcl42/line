#include "LinePair.h"


std::ostream& operator<<(std::ostream& out, const LinePair& linePair)
{
    Line** lines = linePair.getLines();
    out << "<-- LinePair -->" << std::endl;
    
    for(unsigned int i = 0; i < linePair.getLineCount(); i++)
    {
        out << *lines[i] << std::endl;
    }
    return out;
}

