/* 
 * File:   EdgeFilterStrategy.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:18 AM
 */

#ifndef EDGEFILTERSTRATEGY_H
#define	EDGEFILTERSTRATEGY_H

#include "../../../Image/Image.h"
#include "../../../Image/ImageMap.h"


template <class T>
class EdgeFilterStrategy
{
protected:
    Image<T>* m_image;   
    
    double* m_buffer;
    
    double m_min;
    
    double m_max;        
     
    //ImageMap<unsigned int>* m_imageMap;
        
public:
    EdgeFilterStrategy();    
    
    EdgeFilterStrategy(Image<T>* image);    
    
    virtual ~EdgeFilterStrategy();
    
public:
    void setImage(Image<T>* image);
    
    Image<T>* getImage() const;        
    
    virtual void applyFilter(unsigned int threshold) = 0;        
    
protected:                
    void resolveThreshold(unsigned int threshold);       
    
    //void initImageMap(Image<T>* image);
    
};


template <class T>
EdgeFilterStrategy<T>::EdgeFilterStrategy()
: m_image(NULL), m_buffer(NULL), m_min(0), m_max(0)
{ 
}


template <class T>
EdgeFilterStrategy<T>::EdgeFilterStrategy(Image<T>* image)
: m_image(image), m_buffer(NULL), m_min(0), m_max(0)
{
    //initImageMap(image);
}


template <class T>
EdgeFilterStrategy<T>::~EdgeFilterStrategy()
{
    //SAFE_DELETE(m_imageMap);
}


//template <class T>
//void EdgeFilterStrategy<T>::initImageMap(Image<T>* image)
//{        
//    if(m_imageMap == NULL)
//    {
//        m_imageMap = new ImageMap<unsigned int>(image->getWidth(), image->getHeight());
//    }    
//    else if(m_imageMap->getHeight() != image->getHeight() || m_imageMap->getWidth() != image->getWidth())
//    {        
//        SAFE_DELETE(m_imageMap);
//        m_imageMap = new ImageMap<unsigned int>(image->getWidth(), image->getHeight()); 
//    }
//}


template <class T>
void EdgeFilterStrategy<T>::setImage(Image<T>* image)
{
    if(image != NULL)
    {
        m_image = image;
        //initImageMap(image);
    }
    else 
    {
        throw std::runtime_error("EdgeFilterStreategy:setImage -> Invalid Image");
    }
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
    
    for (unsigned int y = 0; y < imageHeight; y++)
    {
        for (unsigned int x = 0; x < imageWidth; x++)
        {
            double val = (m_buffer[y * imageWidth + x] - m_min) / (m_max - m_min) * 255;            

            if (val > threshold)
            {
                pixel.r = 255;
                pixel.g = 255;
                pixel.b = 255;
                //m_imageMap->setValueAt(y, x, 255);
            }
            else
            {
                pixel.r = 0;
                pixel.g = 0;
                pixel.b = 0;
                //m_imageMap->setValueAt(y, x, 0);
            }

            m_image->setPixelValue(y, x, &pixel);            
        }
    }    
}

#endif	/* EDGEFILTERSTRATEGY_H */

