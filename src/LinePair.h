/* 
 * File:   BestLine.h
 * Author: lubos
 *
 * Created on February 22, 2013, 11:56 PM
 */

#ifndef BESTLINE_H
#define	BESTLINE_H

#include "LineDescribableObject.h"


class LinePair : public LineDescribableObject
{
public:
    LinePair() 
    : LineDescribableObject(2) {}
    
    virtual ~LinePair() {}            
    
    friend std::ostream& operator<<(std::ostream& out, const LinePair& linePair);
};

#endif	/* BESTLINE_H */

