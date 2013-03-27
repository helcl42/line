/* 
 * File:   LineToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 8:21 PM
 */

#ifndef LINETOCOMMAND_H
#define	LINETOCOMMAND_H

#include "Command.h"

class LineToCommand : public Command
{      
public:
    LineToCommand() {}

    virtual ~LineToCommand() {}

public:
    void draw(Line* polygon);

    void print();    
};

#endif	/* LINETOCOMMAND_H */

