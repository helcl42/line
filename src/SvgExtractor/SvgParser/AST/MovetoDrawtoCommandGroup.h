/* 
 * File:   MovetoDrawtoCommandGroups.h
 * Author: lubos
 *
 * Created on March 21, 2013, 4:27 PM
 */

#ifndef MOVETODRAWTOCOMMANDGROUPS_H
#define	MOVETODRAWTOCOMMANDGROUPS_H

#include "Node.h"
#include "Commands/Command.h"


class MovetoDrawtoCommandGroup : public Node
{
private:    
    Command* m_moveTo;
    
    std::vector<Command*> m_drawToCommands;
    
public:
    MovetoDrawtoCommandGroup() {}
    
    virtual ~MovetoDrawtoCommandGroup() 
    {
        SAFE_DELETE(m_moveTo);
        std::vector<Command*>::iterator ii;
        for(ii = m_drawToCommands.begin(); ii != m_drawToCommands.end(); ++ii)
        {
            SAFE_DELETE(*ii);
        }
    }
    
public:            
    std::vector<Command*> getDrawToCommands() const
    {
        return m_drawToCommands;
    }

    void setDrawToCommands(std::vector<Command*> drawToCommands)
    {
        this->m_drawToCommands = drawToCommands;
    }

    Command* getMoveTo() const
    {
        return m_moveTo;
    }

    void setMoveTo(Command* moveTo)
    {
        this->m_moveTo = moveTo;
    }
    
    void draw(Line<int>* polygon)
    {
        m_moveTo->draw(polygon);
        
        std::vector<Command*>::iterator ii;                
        for(ii = m_drawToCommands.begin(); ii != m_drawToCommands.end(); ++ii)
        {            
            (*ii)->draw(polygon);
        }
    }
    
    void print() 
    {
        std::cout << "MoveToDrawToCommandGroup" << std::endl;
        m_moveTo->print();
        
        std::vector<Command*>::iterator ii;                
        for(ii = m_drawToCommands.begin(); ii != m_drawToCommands.end(); ++ii)
        {
            (*ii)->print();
        }
    }
};

#endif	/* MOVETODRAWTOCOMMANDGROUPS_H */

