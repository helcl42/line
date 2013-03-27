/* 
 * File:   StraightDetectedObject.h
 * Author: lubos
 *
 * Created on March 10, 2013, 8:36 PM
 */

#ifndef LINEPAIR_H
#define	LINEPAIR_H

#include "../Line.h"
#include "../Utils/Utils.h"
#include "IDetectedObject.h"
#include "../DetectionParams.h"

class LinePair : public IDetectedObject
{
protected:
    Line** m_lines;
    
    unsigned int m_lineCount;
    
public:    
    LinePair();
    
    virtual ~LinePair();
    
public:    
    void setLocked();        
    
    bool isValid();
    
    void invalidate();
    
    void setAt(Line* line, unsigned int index);
    
    Line* getAt(unsigned int index) const;
    
    Line** getPolygons();        
    
    bool hasLengthInInterval(float divider = 10);
    
    unsigned int getCountOfPolygons() const;
    
    Vector2<int>* getObjectPoint();
    
    friend std::ostream& operator<<(std::ostream& out, const LinePair& linePair);
};

#endif	/* STRAIGHTDETECTEDOBJECT_H */

