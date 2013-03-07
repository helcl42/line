/* 
 * File:   EdgeFilterStrategy.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:18 AM
 */

#ifndef EDGEFILTERSTRATEGY_H
#define	EDGEFILTERSTRATEGY_H

#include "Image.h"


template <class T>
class EdgeFilterStrategy
{
protected:
    Image<T>* m_image;   
    
    double* m_buffer;
    
    double m_min;
    
    double m_max;
        
public:
    EdgeFilterStrategy()
    : m_image(NULL), m_buffer(NULL), m_min(0), m_max(0) {}
    
    EdgeFilterStrategy(Image<T>* image)
    : m_image(image), m_buffer(NULL), m_min(0), m_max(0) {}
    
    virtual ~EdgeFilterStrategy() {}
    
public:
    void setImage(Image<T>* image);
    
    Image<T>* getImage() const;        
    
    void resolveThreshold(unsigned int threshold);
    
    virtual void applyFilter(unsigned int threshold) = 0;        
};


template <class T>
void EdgeFilterStrategy<T>::setImage(Image<T>* image)
{
    m_image = image;
}


template <class T>        
Image<T>* EdgeFilterStrategy<T>::getImage() const
{
    return m_image;
}


template <class T>
void EdgeFilterStrategy<T>::resolveThreshold(unsigned int threshold)
{
    PixelRGB<float> pixel;
    unsigned int imageHeight = this->m_image->getHeight();
    unsigned int imageWidth = this->m_image->getWidth();
    
    for (unsigned int y = 1; y < imageHeight - 1; y++)
    {
        for (unsigned int x = 1; x < imageWidth - 1; x++)
        {
            double val = (m_buffer[y * imageWidth + x] - m_min) / (m_max - m_min) * 255;            

            if (val > threshold)
            {
                pixel.r = 255;
                pixel.g = 255;
                pixel.b = 255;
            }
            else
            {
                pixel.r = 0;
                pixel.g = 0;
                pixel.b = 0;
            }

            m_image->setPixelValue(y, x, &pixel);
        }
    }    
}

#endif	/* EDGEFILTERSTRATEGY_H */

