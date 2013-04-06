/* 
 * File:   SobelFilterStrategy.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:23 AM
 */

#ifndef SOBELFILTERSTRATEGY_H
#define	SOBELFILTERSTRATEGY_H

#include "EdgeFilterStrategy.h"

template <class T>
class SobelFilterStrategy : public EdgeFilterStrategy<T>
{   
public:
    SobelFilterStrategy() 
        : EdgeFilterStrategy<T>()  {}
    
    SobelFilterStrategy(Image<T>* image)
        : EdgeFilterStrategy<T>(image) {}
       
    virtual ~SobelFilterStrategy() {}
         
    void applyFilter(unsigned int treshold);    
    
private:    
    void sobelAlgorithm();
};

template <class T>
void SobelFilterStrategy<T>::sobelAlgorithm()
{
    this->m_min = 1.0;
    this->m_max = 0.0;    

    unsigned int imageHeight = this->m_image->getHeight();
    unsigned int imageWidth = this->m_image->getWidth();

    this->m_buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i += 1)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j += 1)
        {
            double gx =
                    Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1),
                    this->m_image->getPixel(i + 1, j - 1)) +
                    2.0 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j),
                    this->m_image->getPixel(i + 1, j)) +
                    Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j + 1),
                    this->m_image->getPixel(i + 1, j + 1));

            double gy =
                    Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1),
                    this->m_image->getPixel(i - 1, j + 1)) +
                    2.0 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j - 1),
                    this->m_image->getPixel(i, j + 1)) +
                    Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j - 1),
                    this->m_image->getPixel(i + 1, j + 1));

            double val = pow(gx * gx + gy * gy, 0.5);

            if (val > this->m_max) this->m_max = val;
            if (val < this->m_min) this->m_min = val;

            this->m_buffer[i * imageWidth + j] = val;
        }
    }
}


template <class T>
void SobelFilterStrategy<T>::applyFilter(unsigned int threshold)
{    
    sobelAlgorithm();
    this->resolveThreshold(threshold);
    SAFE_DELETE_ARRAY(this->m_buffer);    
}

#endif	/* SOBELFILTERSTRATEGY_H */

