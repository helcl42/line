/* 
 * File:   GaussianFactory.h
 * Author: lubos
 *
 * Created on March 31, 2013, 12:00 AM
 */

#ifndef GAUSSIANFACTORY_H
#define	GAUSSIANFACTORY_H

#include "Kernel/Kernel.h"
#include "Kernel/NonSeparableKernel.h"

template <class T>
class GaussianFactory
{
public:
    static NonSeparableKernel<T>* createGaussianKernel(double alpha, double sigma)
    {
        int width = std::ceil(alpha * sigma);
        T* res = new T[width * 2 + 1];
        
        double coeff;
        
        for(unsigned int i = 0; i <= width; i++)
        {
            coeff = std::pow(M_E, (i * i) / 2 * sigma * sigma) / std::sqrt(2 * M_PI * sigma);
            res[width + 1 + i] = coeff;
            res[width + 1 - i] = coeff;
        }        
        
        NonSeparableKernel<T>* ret = new NonSeparableKernel<T>(width, width, res);
        SAFE_DELETE(res);
        return ret;
    }
};

#endif	/* GAUSSIANFACTORY_H */

