/* 
 * File:   SmoothQuadraticBezerCurveToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:26 PM
 */

#ifndef SMOOTHQUADRATICBEZERCURVETOCOMMAND_H
#define	SMOOTHQUADRATICBEZERCURVETOCOMMAND_H
        
#include "Command.h"
        
class SmoothQuadraticBezierCurveToCommand : public Command
{
private:
    
public:        
    SmoothQuadraticBezierCurveToCommand() {}
    
    virtual ~SmoothQuadraticBezierCurveToCommand() {}
    
public:    
    void draw(Line<int>* polygon)
    {
    }
    
    void print() 
    {
        std::cout << "smoothQuadraticBezierCurveCommand" << std::endl;
    }
};

#endif	/* SMOOTHQUADRATICBEZERCURVETOCOMMAND_H */

