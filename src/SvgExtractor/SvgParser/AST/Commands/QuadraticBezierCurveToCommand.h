/* 
 * File:   QuadraticBezierCurveToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:20 PM
 */

#ifndef QUADRATICBEZIERCURVETOCOMMAND_H
#define	QUADRATICBEZIERCURVETOCOMMAND_H
        
#include "Command.h"

class QuadraticBezierCurveToCommand : public Command
{
private:
    
public:        
    QuadraticBezierCurveToCommand() {}
    
    virtual ~QuadraticBezierCurveToCommand() {}
    
public:    
    void draw(Line<int>* polygon)
    {
    }
    
    void print() 
    {
        std::cout << "quadraticBezierCommand" << std::endl;
    }
};

#endif	/* QUADRATICBEZIERCURVETOCOMMAND_H */

