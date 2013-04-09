/* 
 * File:   HorizontalLineToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 8:26 PM
 */

#ifndef HORIZONTALLINETOCOMMAND_H
#define	HORIZONTALLINETOCOMMAND_H

#include "Command.h"

class HorizontalLineToCommand : public Command
{
private:
    
public:        
    HorizontalLineToCommand() {}
    
    virtual ~HorizontalLineToCommand() {}
    
public:    
    void draw(Line<int>* polygon)
    {
    }
    
    void print() 
    {
        std::cout << "horizontalLineToCommand" << std::endl;
    }
};

#endif	/* HORIZONTALLINETOCOMMAND_H */

