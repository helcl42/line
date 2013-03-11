/* 
 * File:   BestLine.h
 * Author: lubos
 *
 * Created on February 22, 2013, 11:56 PM
 */

#ifndef BESTLINE_H
#define	BESTLINE_H

#include "StraightDetectedObject.h"


class LinePair : public StraightDetectedObejct
{
public:
    LinePair() 
    : StraightDetectedObejct(2) {}
    
    virtual ~LinePair() {}
};

#endif	/* BESTLINE_H */

