/* 
 * File:   BestLine.h
 * Author: lubos
 *
 * Created on February 22, 2013, 11:56 PM
 */

#ifndef BESTLINE_H
#define	BESTLINE_H

#include "Line.h"
#include "Utils.h"


class BestLine
{
protected:
    Line* line[2];
    
public:    
    BestLine() 
    {        
        line[0] = NULL;
        line[1] = NULL;
    }
    
    ~BestLine() {}
    
    bool isValid() const
    {
        return line[0] != NULL && line[1] != NULL;
    }
    
    Line* getFirst() const 
    {
        return line[0];
    }
    
    Line* getSecond() const 
    {
        return line[1];
    }
    
    void setLine(Line* l1, Line* l2) 
    {
        line[0] = l1;
        line[1] = l2;
    }
    
    void invalidate() 
    {
        line[0] = NULL;
        line[1] = NULL;
    }
};

#endif	/* BESTLINE_H */

