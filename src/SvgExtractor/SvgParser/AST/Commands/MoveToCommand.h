/* 
 * File:   MoveToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 7:09 PM
 */

#ifndef MOVETOCOMMAND_H
#define	MOVETOCOMMAND_H

#include "Command.h"
#include "../PositionManager.h"

class MoveToCommand : public Command
{
private:
    CoordinatePair* m_startPoint;

public:
    MoveToCommand();

    virtual ~MoveToCommand();

public:

    void draw(Polygon<int>* polygon);

    void print();
    
    CoordinatePair* getStartPoint() const;

    void setStartPoint(CoordinatePair* startPoint);
};

#endif	/* MOVETOCOMMAND_H */

