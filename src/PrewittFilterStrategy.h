/* 
 * File:   PrewittFilterStrategy.h
 * Author: lubos
 *
 * Created on March 6, 2013, 3:14 PM
 */

#ifndef PREWITTFILTERSTRATEGY_H
#define	PREWITTFILTERSTRATEGY_H


#include "EdgeFilterStrategy.h"


template <class T>
class PrewittFilterStrategy : public EdgeFilterStrategy<T>
{   
public:
    PrewittFilterStrategy()
        : EdgeFilterStrategy<T>()  {}
    
    PrewittFilterStrategy(Image<T>* image)
        : EdgeFilterStrategy<T>(image) {}
       
    virtual ~PrewittFilterStrategy() {}
         
    void applyFilter(unsigned int threshold);    

private:    
    void prewittAlgorithm();
};


/**
 *    | 1   1   0 | 
 *    | 1   0  -1 |
 *    | 0  -1  -1 | 
 */
template <class T>
void PrewittFilterStrategy<T>::prewittAlgorithm()
{
    this->m_min = 1.0;
    this->m_max = 0.0;
    const double k = 1 /3;

    unsigned int imageHeight = this->m_image->getHeight();
    unsigned int imageWidth = this->m_image->getWidth();

    m_buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i += 1)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j += 1)
        {
            double gx = k *
                    Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1), this->m_image->getPixel(i + 1, j - 1)) +
                    Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j    ), this->m_image->getPixel(i + 1, j    )) +
                    Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j + 1), this->m_image->getPixel(i + 1, j + 1));

            double gy = k *
                    Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1), this->m_image->getPixel(i - 1, j + 1)) +
                    Pixel<float>::colourDifference(this->m_image->getPixel(i    , j - 1), this->m_image->getPixel(i    , j + 1)) +
                    Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j - 1), this->m_image->getPixel(i + 1, j + 1));

            double val = pow(gx * gx + gy * gy, 0.5);

            if (val > this->m_max) this->m_max = val;
            if (val < this->m_min) this->m_min = val;

            this->m_buffer[i * imageWidth + j] = val;
        }
    }
}


template <class T>
void PrewittFilterStrategy<T>::applyFilter(unsigned int threshold)
{ 
    prewittAlgorithm();
    this->resolveThreshold(threshold);
    SAFE_DELETE_ARRAY(buffer);
}

#endif	/* PREWITTFILTERSTRATEGY_H */

