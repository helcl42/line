/* 
 * File:   Flag.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:49 PM
 */

#ifndef FLAG_H
#define	FLAG_H

#include "Argument.h"

class Flag : public Argument
{
private:    
    bool m_value;
    
public:    
    Flag() : Argument(FLAG) {}
    
    virtual ~Flag() {}
    
public:    
    bool IsValue() const
    {
        return m_value;
    }

    void SetValue(bool value)
    {
        this->m_value = value;
    }    
    
    void draw(Line<int>* polygon)
    {
    }
    
    void print() 
    {
        std::cout << m_value;
    }
};

#endif	/* FLAG_H */

