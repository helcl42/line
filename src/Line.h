/* 
 * File:   Line.h
 * Author: lubos
 *
 * Created on February 10, 2013, 1:21 AM
 */

#ifndef LINE_H
#define	LINE_H

#include "Vector2.h"

struct Line
{
    std::vector<Vector2<int> > points;
    
    Line() {}
    ~Line() {}
    
    //TODO compute approx direction etc
    
    friend std::ostream& operator<< (std::ostream& out, const Line& line) 
    {
        out << "Line len = " << line.points.size() << std::endl;
        
        std::vector<Vector2<int> >::const_iterator ii;
        for(ii = line.points.begin(); ii != line.points.end(); ++ii) 
        {
            out << *ii << " ";
        }       
        return out;
    }
};

#endif	/* LINE_H */

