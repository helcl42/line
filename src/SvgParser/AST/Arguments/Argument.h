/* 
 * File:   Argument.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:46 PM
 */

#ifndef ARGUMENT_H
#define	ARGUMENT_H

#include "../Node.h"
#include "ArgumentType.h"

class Argument : public Node
{
protected:   
    ArgumentType m_argumentType;
    
public:
    Argument(ArgumentType argType)
        : m_argumentType(argType) {}
    
    virtual ~Argument() {}  
    
public:    
    ArgumentType GetArgumentType() const;
    
    void SetArgumentType(ArgumentType argumentType);    
};

#endif	/* ARGUMENT_H */

