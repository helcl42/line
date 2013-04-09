/* 
 * File:   Coordinate.h
 * Author: lubos
 *
 * Created on March 23, 2013, 6:16 PM
 */

#ifndef COORDINATE_H
#define	COORDINATE_H

#include "Argument.h"

class Coordinate : public Argument
{
private:    
    float m_value;    

public:    
    Coordinate() 
        : Argument(COORDINATE), m_value(0) {}
    
    virtual ~Coordinate() {}
    
public:               
    float getValue() const;

    void setValue(float value);    
    
    void draw(Line<int>* polygon);
    
    void print();     
};

#endif	/* COORDINATE_H */

