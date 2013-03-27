/* 
 * File:   MoveToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 7:09 PM
 */

#ifndef MOVETOCOMMAND_H
#define	MOVETOCOMMAND_H

#include "Command.h"
#include "../PositionFactory.h"

class MoveToCommand : public Command
{
private:
    CoordinatePair* m_startPoint;

public:
    MoveToCommand();

    virtual ~MoveToCommand();

public:

    void draw(Line* polygon);

    void print();
    
    CoordinatePair* getStartPoint() const;

    void setStartPoint(CoordinatePair* startPoint);
};

#endif	/* MOVETOCOMMAND_H */

