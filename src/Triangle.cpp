#include "Triangle.h"

std::ostream& operator<<(std::ostream& out, const Triangle& triangle)
{
    Line** lines = triangle.getLines();
    out << "<-- Triangle -->" << std::endl;
    
    for(unsigned int i = 0; i < triangle.getLineCount(); i++)
    {
        out << *lines[i] << std::endl;
    }
    return out;
}
