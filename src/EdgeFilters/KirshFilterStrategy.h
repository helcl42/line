/* 
 * File:   KirshFilterStrategy.h
 * Author: lubos
 *
 * Created on March 6, 2013, 3:06 PM
 */

#ifndef KIRSHFILTERSTRATEGY_H
#define	KIRSHFILTERSTRATEGY_H

#include "EdgeFilterStrategy.h"


template <class T>
class KirshFilterStrategy : public EdgeFilterStrategy<T>
{   
public:
    KirshFilterStrategy()
        : EdgeFilterStrategy<T>()  {}
    
    KirshFilterStrategy(Image<T>* image)
        : EdgeFilterStrategy<T>(image) {}
       
    virtual ~KirshFilterStrategy() {}
         
    void applyFilter(unsigned int threshold);    
    
private:
    void kirshAlgorithm();
};


/** 
 *  |  5   5  -3 | 
 *  |  5   0  -3 |
 *  | -3  -3  -3 |    
 */
template <class T>
void KirshFilterStrategy<T>::kirshAlgorithm()
{
    this->m_min = 1.0;
    this->m_max = 0.0;
    const double k = 1/15;

    unsigned int imageHeight = this->m_image->getHeight();
    unsigned int imageWidth = this->m_image->getWidth();

    m_buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i++)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j++)
        {
            double gx = k *                    
                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1), this->m_image->getPixel(i, j)) 
                  - 5 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j    ), this->m_image->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j + 1), this->m_image->getPixel(i, j))
                  + 5 * Pixel<float>::colourDifference(this->m_image->getPixel(i    , j - 1), this->m_image->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i    , j + 1), this->m_image->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j - 1), this->m_image->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j    ), this->m_image->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j + 1), this->m_image->getPixel(i, j));

            double gy = k *
                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1), this->m_image->getPixel(i, j)) 
                  + 5 * Pixel<float>::colourDifference(this->m_image->getPixel(i    , j - 1), this->m_image->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j - 1), this->m_image->getPixel(i, j)) 
                  - 5 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j    ), this->m_image->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j    ), this->m_image->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j + 1), this->m_image->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i    , j + 1), this->m_image->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j + 1), this->m_image->getPixel(i, j));

            double val = pow(gx * gx + gy * gy, 0.5);

            if (val > this->m_max) this->m_max = val;
            if (val < this->m_min) this->m_min = val;

            this->m_buffer[i * imageWidth + j] = val;
        }
    }
}


template <class T>
void KirshFilterStrategy<T>::applyFilter(unsigned int threshold)
{
    kirshAlgorithm();
    this->resolveThreshold(threshold);
    SAFE_DELETE_ARRAY(buffer);
}


#endif	/* KIRSHFILTERSTRATEGY_H */

