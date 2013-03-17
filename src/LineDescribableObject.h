/* 
 * File:   StraightDetectedObject.h
 * Author: lubos
 *
 * Created on March 10, 2013, 8:36 PM
 */

#ifndef STRAIGHTDETECTEDOBJECT_H
#define	STRAIGHTDETECTEDOBJECT_H

#include "IDetectedObject.h"
#include "DetectionParams.h"
#include "Utils.h"

class LineDescribableObject : public IDetectedObject
{
protected:
    Line** m_lines;
    
    unsigned int m_lineCount;
    
public:    
    LineDescribableObject(unsigned int lineCount);
    
    virtual ~LineDescribableObject();
    
public:    
    void setLocked();        
    
    void setLocked(unsigned int count);
    
    bool isValid() const;
    
    bool isValid(unsigned int count) const;
    
    void invalidate();
    
    void setAt(Line* line, unsigned int index);
    
    Line* getAt(unsigned int index) const;
    
    Line** getLines() const;
    
    bool hasLengthInInterval(float divider = 10);
    
    unsigned int getLineCount() const;
};

#endif	/* STRAIGHTDETECTEDOBJECT_H */

