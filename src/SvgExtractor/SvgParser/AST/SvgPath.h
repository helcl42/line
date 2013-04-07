/* 
 * File:   SvgPath.h
 * Author: lubos
 *
 * Created on March 21, 2013, 4:23 PM
 */

#ifndef SVGPATH_H
#define	SVGPATH_H

#include "Node.h"

class SvgPath : public Node
{
private:
    std::vector<Node*> m_movetoDrawtoCommandGroups;

public:
    SvgPath() {}

    virtual ~SvgPath() 
    {
        std::vector<Node*>::iterator ii;
        for(ii = m_movetoDrawtoCommandGroups.begin(); ii != m_movetoDrawtoCommandGroups.end(); ++ii)
        {
            SAFE_DELETE(*ii);
        }
    }

public:

    void setMovetoDrawtoCommandGroups(std::vector<Node*> moves)
    {
        m_movetoDrawtoCommandGroups = moves;
    }

    std::vector<Node*> getMovetoDrawtoCommandGroups() const
    {
        return m_movetoDrawtoCommandGroups;
    }

    void draw(Line* polygon)
    {
        std::cout << "------------ DRAW ------------" << std::endl;
        
        std::vector<Node*>::iterator ii;
        for (ii = m_movetoDrawtoCommandGroups.begin(); ii != m_movetoDrawtoCommandGroups.end(); ++ii)
        {            
            (*ii)->draw(polygon);
        }
    }

    void print()
    {
        std::cout << "------------ PRINT ------------" << std::endl;

        std::vector<Node*>::iterator ii;
        for (ii = m_movetoDrawtoCommandGroups.begin(); ii != m_movetoDrawtoCommandGroups.end(); ++ii)
        {
            (*ii)->print();
        }
    }
};

#endif	/* SVGPATH_H */

