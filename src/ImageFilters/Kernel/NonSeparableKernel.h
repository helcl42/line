/* 
 * File:   NonSeparableKernel.h
 * Author: lubos
 *
 * Created on March 30, 2013, 11:36 PM
 */

#ifndef NONSEPARABLEKERNEL_H
#define	NONSEPARABLEKERNEL_H

#include "../../Utils/Utils.h"
#include "Kernel.h"

template <class T> class NonSeparableKernel;
template <class T> std::ostream& operator<<(std::ostream& out, const NonSeparableKernel<T>& ker);

template <class T>
class NonSeparableKernel : public Kernel<T>
{
private:
    T* m_values;

    float m_sum;

public:

    NonSeparableKernel(unsigned int width, unsigned int height)
    : Kernel<T>(width, height), m_sum(0)
    {
        unsigned int size = width * height;
        m_values = new T[size];
        for (unsigned int i = 0; i < size; i++)
        {
            m_values[i] = 0;
        }
    }

    NonSeparableKernel(unsigned int width, unsigned int height, T* values)
    : Kernel<T>(width, height), m_sum(0)
    {
        double res = 0;
        unsigned int size = width * height;
        m_values = new T[size];

        for (unsigned int i = 0; i < size; i++)
        {
            m_values[i] = values[i];
            res += std::abs(values[i]);
        }

        m_sum = res >= size ? res : 1;
        normalize();
    }

    virtual ~NonSeparableKernel()
    {
        SAFE_DELETE_ARRAY(m_values);
    }

public:

    bool isSeparable() const
    {
        return false;
    }

    Kernel<T>* getColVector() const
    {
        throw std::runtime_error("Not implemented method: getColVector in NonSeparableKernel");
    }

    Kernel<T>* getRowVector() const
    {
        throw std::runtime_error("Not implemented method: getRowVector in NonSeparableKernel");
    }

    T getValue(unsigned int y, unsigned int x) const
    {
        if (y < this->m_height && y >= 0 && x < this->m_width && x >= 0)
        {
            return m_values[y * this->m_width + x];
        }
        else
        {
            throw std::runtime_error("NonSeparableKernel:getValue -> Index out");
        }
    }

    Kernel<T>& operator[](int index)
    {
        throw std::runtime_error("NonseparableKernel:operator[] -> invalid operation");
    }

    friend std::ostream& operator<<(std::ostream& out, const NonSeparableKernel<T>& ker)
    {
        out << "NonseparableKernel" << std::endl;
        out << "{" << std::endl;
        for (unsigned int i = 0; i < ker.m_height; i++)
        {
            out << "\t";
            for (unsigned int j = 0; j < ker.m_width; j++)
            {
                out << ker.m_values[i * ker.m_width + j] << ", ";
            }
            out << "\n";
        }
        out << "}";
        return out;
    }

private:

    void normalize() const
    {
        std::cout << "SUM = " << m_sum << std::endl;
        for (unsigned int i = 0; i < this->m_height; i++)
        {
            for (unsigned int j = 0; j < this->m_width; j++)
            {
                m_values[i * this->m_width + j] /= m_sum;
            }
        }
    }
};

#endif	/* NONSEPARABLEKERNEL_H */

