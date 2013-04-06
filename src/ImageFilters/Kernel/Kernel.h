/* 
 * File:   Kernel.h
 * Author: lubos
 *
 * Created on March 30, 2013, 11:07 PM
 */

#ifndef KERNEL_H
#define	KERNEL_H

#include "../../Utils/Utils.h"

template <class T> class Kernel;

template <class T>
class Kernel
{
protected:
    unsigned int m_width;

    unsigned int m_height;    
    
public:

    Kernel(unsigned int width, unsigned int height)
    : m_width(width), m_height(height) {}  

    virtual ~Kernel() {}

    unsigned int getHeight() const
    {
        return m_height;
    }

    void getHeight(unsigned int height)
    {
        this->m_height = height;
    }
   
    unsigned int getWidth() const
    {
        return m_width;
    }

    void setWidth(unsigned int width)
    {
        this->m_width = width;
    }
    
public:    
    virtual bool isSeparable() const = 0;
    
    virtual T getValue(unsigned int y, unsigned int x) const = 0; 
    
    virtual Kernel<T>* getRowVector() const = 0;
    
    virtual Kernel<T>* getColVector() const = 0;    
    
    virtual Kernel<T>& operator[](int index) = 0;    
};

#endif	/* KERNEL_H */

