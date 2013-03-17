/* 
 * File:   RobertsFilterStrategy.h
 * Author: lubos
 *
 * Created on March 6, 2013, 3:22 PM
 */

#ifndef ROBERTSFILTERSTRATEGY_H
#define	ROBERTSFILTERSTRATEGY_H

#include "RobertsStrategy.h"


#include "EdgeFilterStrategy.h"


template <class T>
class RobertsFilterStrategy : public EdgeFilterStrategy<T>
{   
public:
    RobertsFilterStrategy()
        : EdgeFilterStrategy<T>()  {}
    
    RobertsFilterStrategy(Image<T>* image)
        : EdgeFilterStrategy<T>(image) {}
       
    virtual ~RobertsFilterStrategy() {}
         
    ImageMap<unsigned int>* applyFilter(unsigned int threshold);    
    
private:    
    void robertsAlgorithm();
};


/** 
 *    | 0   0   0 | 
 *  X | 0   1   0 |
 *    | 0   0  -1 | 
 * 
 *    | 0   0   0 | 
 *  Y | 0   0   1 |
 *    | 0  -1   0 | 
 */
template <class T>
void RobertsFilterStrategy<T>::robertsAlgorithm()
{
    this->m_min = 1.0;
    this->m_max = 0.0;

    unsigned int imageHeight = this->m_image->getHeight();
    unsigned int imageWidth = this->m_image->getWidth();

    this->m_buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i++)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j++)
        {            
            double gx = Pixel<float>::colourDifference(this->m_image->getPixel(i, j), this->m_image->getPixel(i, j + 1)) - Pixel<float>::colourDifference(this->m_image->getPixel(i, j    ), this->m_image->getPixel(i - 1, j    ));

            double gy = Pixel<float>::colourDifference(this->m_image->getPixel(i, j), this->m_image->getPixel(i, j    )) - Pixel<float>::colourDifference(this->m_image->getPixel(i, j + 1), this->m_image->getPixel(i + 1, j + 1));                        

            double val = pow(gx * gx + gy * gy, 0.5);

            if (val > this->m_max) this->m_max = val;
            if (val < this->m_min) this->m_min = val;

            this->m_buffer[i * imageWidth + j] = val;
        }
    }
}

template <class T>
ImageMap<unsigned int>* RobertsFilterStrategy<T>::applyFilter(unsigned int threshold)
{
    robertsAlgorithm();
    this->resolveThreshold(threshold);
    SAFE_DELETE_ARRAY(buffer);
    return m_imageMap;
}

#endif	/* ROBERTSFILTERSTRATEGY_H */

