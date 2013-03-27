/* 
 * File:   MoveTo.h
 * Author: lubos
 *
 * Created on March 21, 2013, 6:00 PM
 */

#ifndef MOVETO_H
#define	MOVETO_H

#include "../Arguments/Argument.h"


class Command : public Node
{
protected:    
    std::vector<Argument*> m_args;
    
    bool m_relative;
    
public:
    Command() : m_relative(true) {}
    
    virtual ~Command() 
    {
        std::vector<Argument*>::iterator ii;
        for(ii = m_args.begin(); ii != m_args.end(); ++ii)
        {
            SAFE_DELETE(*ii);
        }
    }
    
public:       
    std::vector<Argument*> getArgs() const;

    void setArgs(std::vector<Argument*> args);    

    bool isRelative() const;

    void setRelative(bool relative);    
};

#endif	/* MOVETO_H */

