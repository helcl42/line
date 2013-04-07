/* 
 * File:   VerticalLineToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 8:34 PM
 */

#ifndef VERTICALLINETOCOMMAND_H
#define	VERTICALLINETOCOMMAND_H

#include "Command.h"


class VerticalLineToCommand : public Command
{
private:
    
public:        
    VerticalLineToCommand() {}
    
    virtual ~VerticalLineToCommand() {}
    
public:    
    void draw(Line* polygon)
    {
    }        
    
    void print()
    {
        std::cout << "lineToCommand" << std::endl;
    }
};

#endif	/* VERTICALLINETOCOMMAND_H */

