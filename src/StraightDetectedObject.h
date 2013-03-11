/* 
 * File:   StraightDetectedObject.h
 * Author: lubos
 *
 * Created on March 10, 2013, 8:36 PM
 */

#ifndef STRAIGHTDETECTEDOBJECT_H
#define	STRAIGHTDETECTEDOBJECT_H

#include "DetectedObject.h"
#include "Line.h"
#include "Utils.h"

class StraightDetectedObejct : public DetectedObject
{
protected:
    Line** m_lines;
    
    unsigned int m_lineCount;
    
public:    
    StraightDetectedObejct(unsigned int lineCount);
    
    virtual ~StraightDetectedObejct();
    
public:    
    void setLocked();        
    
    void setLocked(unsigned int count);
    
    bool isValid() const;
    
    bool isValid(unsigned int count) const;
    
    void invalidate();
    
    void setAt(Line* line, unsigned int index);
    
    Line* getAt(unsigned int index) const;
    
    Line** getLines();
    
    unsigned int getLineCount() const;
};

#endif	/* STRAIGHTDETECTEDOBJECT_H */

