/* 
 * File:   BestLine.h
 * Author: lubos
 *
 * Created on February 22, 2013, 11:56 PM
 */

#ifndef BESTLINE_H
#define	BESTLINE_H

#include "StraightDetectedObject.h"


class LinePair : public StraightDetectedObject
{
public:
    LinePair() 
    : StraightDetectedObject(2) {}
    
    virtual ~LinePair() {}            
    
    friend std::ostream& operator<<(std::ostream& out, const LinePair& linePair);
};

#endif	/* BESTLINE_H */

