/* 
 * File:   SeparableKernel.h
 * Author: lubos
 *
 * Created on March 30, 2013, 11:34 PM
 */

#ifndef SEPARABLEKERNEL_H
#define	SEPARABLEKERNEL_H

#include "Utils.h"
#include "Kernel.h"
#include "NonSeparableKernel.h"

template <class T> class SeparableKernel;
template <class T> std::ostream& operator<<(std::ostream& out, const SeparableKernel<T>& ker);

template <class T>
class SeparableKernel : public Kernel<T>
{
private:
    Kernel<T>* m_VKernel;

    Kernel<T>* m_HKernel;

public:

    SeparableKernel(unsigned int width, unsigned int height, float* VValues, float* HValues)
    : Kernel<T>(width, height)
    {
        m_VKernel = new NonSeparableKernel<T > (width, height, VValues);
        m_HKernel = new NonSeparableKernel<T > (width, height, HValues);
    }

    virtual ~SeparableKernel()
    {
        SAFE_DELETE(m_HKernel);
        SAFE_DELETE(m_VKernel);
    }

    bool isSeparable() const
    {
        return true;
    }

    Kernel<T>* getColVector() const
    {
        return m_HKernel;
    }

    Kernel<T>* getRowVector() const
    {
        return m_VKernel;
    }

    T getValue(unsigned int y, unsigned int x) const
    {
        throw std::runtime_error("Not implemented method: getValue in NonSeparableKernel");
    }

    Kernel<T>& operator[](int index)
    {
        switch (index)
        {
            case 0:
                return m_VKernel;
            case 1:
                return m_HKernel;
            default:
                throw std::runtime_error("SeparableKernel:operator[] -> invalid index");
        }
    }

    float getSumOfElements() const
    {
        float res = 0;
        float tempRes = 0;        
        
        for(unsigned int i = 0; i < this->m_height; i++)
        {
            for(unsigned int j = 0; j < this->m_width; j++)
            {
                tempRes += std::abs(m_HKernel->getValue(i, j));
                if(tempRes > res) res = tempRes;
            }
        }
        return res;
    }
    
    friend std::ostream& operator<<(std::ostream& out, const SeparableKernel<T>& ker)
    {
        out << "SeparableKernel" << std::endl;
        out << "H" << std::endl;
        out << ker.m_HKernel << std::endl;
        out << "V" << std::endl;
        out << ker.m_VKernel << std::endl;
        return out;
    }
};

#endif	/* SEPARABLEKERNEL_H */


