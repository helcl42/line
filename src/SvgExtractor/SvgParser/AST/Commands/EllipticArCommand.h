/* 
 * File:   EllipticArCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:31 PM
 */

#ifndef ELLIPTICARCOMMAND_H
#define	ELLIPTICARCOMMAND_H

#include "Command.h"

class EllipticArcCommand : public Command
{
private:
    
public:        
    EllipticArcCommand() {}
    
    virtual ~EllipticArcCommand() {}
    
public:    
    void draw(Line<int>* polygon)
    {
    }
    
    void print() 
    {
        std::cout << "ellipticArcCommand" << std::endl;
    }
};

#endif	/* ELLIPTICARCOMMAND_H */

