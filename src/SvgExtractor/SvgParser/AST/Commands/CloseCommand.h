/* 
 * File:   CloseCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 7:50 PM
 */

#ifndef CLOSECOMMAND_H
#define	CLOSECOMMAND_H

#include "Command.h"

class CloseCommand : public Command
{
public:
    CloseCommand() {}

    virtual ~CloseCommand() {}

public:

    void draw(Line<int>* polygon);

    void print();
};

#endif	/* CLOSECOMMAND_H */

