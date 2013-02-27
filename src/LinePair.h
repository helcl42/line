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


class LinePair
{
protected:
    Line* m_line[2];
    
public:    
    LinePair();
    
    ~LinePair() {}
    
public:
    bool isValid() const;
    
    Line* getFirst() const;
    
    Line* getSecond() const;
    
    void setLine(Line* l1, Line* l2);
    
    void invalidate();
    
    Line* operator[](int index);
};

#endif	/* BESTLINE_H */

