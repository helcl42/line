/* 
 * File:   SmoothCurveToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:11 PM
 */

#ifndef SMOOTHCURVETOCOMMAND_H
#define	SMOOTHCURVETOCOMMAND_H

#include "Command.h"

class SmoothCurveCommand : public Command
{
private:
    
public:        
    SmoothCurveCommand() {}
    
    virtual ~SmoothCurveCommand() {}
    
public:    
    void draw(Line<int>* polygon)
    {
    }
    
    void print() 
    {
        std::cout << "smoothCurveCommand" << std::endl;
    }
};

#endif	/* SMOOTHCURVETOCOMMAND_H */

