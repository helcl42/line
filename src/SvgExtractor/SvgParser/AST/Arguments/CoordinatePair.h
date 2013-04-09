/* 
 * File:   NumberPair.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:57 PM
 */

#ifndef COORDINATEPAIR_H
#define	COORDINATEPAIR_H

#include "Coordinate.h"
#include "ArgumentType.h"

class CoordinatePair : public Argument
{
private:    
    Coordinate* m_numberX;
    
    Coordinate* m_numberY;

public:    
    CoordinatePair() 
        : Argument(COORDINATE_PAIR), m_numberX(0), m_numberY(0) {}
    
    virtual ~CoordinatePair() 
    {
        SAFE_DELETE(m_numberX);
        SAFE_DELETE(m_numberY);
    }
    
public:    
    Coordinate* GetNumberX() const;

    void SetNumberX(Coordinate* numberX);

    Coordinate* GetNumberY() const;

    void SetNumberY(Coordinate* numberY);
    
    void draw(Line<int>* polygon);
    
    void print();
};

#endif	/* NUMBERPAIR_H */

