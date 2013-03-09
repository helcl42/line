/* 
 * File:   Rectangle.h
 * Author: lubos
 *
 * Created on February 27, 2013, 5:35 PM
 */

#ifndef RECTANGLE_H
#define	RECTANGLE_H

#include <stdexcept>
#include "Line.h"


class Rectangle
{
protected:
    Line* m_line[4];
    
public:    
    Rectangle();
    
    ~Rectangle() {}
    
public:
    bool isValid(unsigned int a = 4) const;
    
    void setLocked(unsigned int a = 4);
        
    void setInstance(Line** lines);
    
    void invalidate();
    
    void setAt(Line* line, unsigned int index);
    
    Line* getAt(unsigned int index) const;
    
    Line* operator[](int index);
};

#endif	/* RECTANGLE_H */

