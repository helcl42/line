/* 
 * File:   svgTraits.h
 * Author: lubos
 *
 * Created on March 21, 2013, 3:58 AM
 */

#ifndef NODE_H
#define	NODE_H

#include <cstdlib>
#include "../../Line.h"
#include "../../Utils/Utils.h"

class Node
{
protected:    
    const static float RATIO_DELTA;
    
public:
    Node() {}
    
    virtual ~Node() {}
    
public:         
    virtual void draw(Line* polygon) = 0;
    
    virtual void print() = 0;    
};

#endif	/* NODE_H */

