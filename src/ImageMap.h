/* 
 * File:   ImageMap.h
 * Author: lubos
 *
 * Created on March 16, 2013, 5:39 PM
 */

#ifndef IMAGEMAP_H
#define	IMAGEMAP_H

#include "Utils/Utils.h"
#include "Image.h"


template <class T>
class ImageMap
{
private:
    T** m_map;
    
    unsigned int m_width;

    unsigned int m_height;
    
public:
    ImageMap();
    
    ImageMap(Image<float>* image);
    
    ImageMap(unsigned int h, unsigned int w);
    
    ~ImageMap();
    
    T** getMapMatrix() const;
    
    T getValueAt(unsigned int y, unsigned int x);
    
    void setValueAt(unsigned int y, unsigned int x, T value);        
    
    unsigned int getWidth() const;
    
    unsigned int getHeight() const;
   
    void setImage(Image<float>* image);
    
protected:    
    void initImageMap(Image<float>* image);
    
private:    
    void cleanUp();
    
    void allocateMap(Image<float>* image);
};

template <class T>
ImageMap<T>::ImageMap()
: m_map(NULL), m_width(0), m_height(0)
{
}

template <class T>
ImageMap<T>::ImageMap(unsigned int h, unsigned int w)
: m_height(h), m_width(w)
{
    m_map = new T* [m_height];
    for(unsigned int i = 0; i < m_height; i++)
    {
        m_map[i] = new T[m_width];
    }
}

template <class T>
ImageMap<T>::ImageMap(Image<float>* image)
: m_width(image->getWidth()), m_height(image->getHeight())
{        
    m_map = new T* [m_height];
    for(unsigned int i = 0; i < m_height; i++)
    {
        m_map[i] = new T[m_width];
    }
    initImageMap(image);
}

template <class T>
void ImageMap<T>::cleanUp()
{
    for(unsigned int i = 0; i < m_height; i++)
    {
        SAFE_DELETE_ARRAY(m_map[i]);
    }
    SAFE_DELETE_ARRAY(m_map);
}

template <class T>
ImageMap<T>::~ImageMap()
{
    cleanUp();
}

template <class T>
void ImageMap<T>::setImage(Image<float>* image)
{    
    if(m_map == NULL)
    {
        allocateMap(image);
    }    
    else if(m_height != image->getHeight() || m_width != image->getWidth())
    {                
        cleanUp();        
        allocateMap(image);
    }
    
    initImageMap(image); 
}

template <class T>
void ImageMap<T>::allocateMap(Image<float>* image)
{
    m_height = image->getHeight();
    m_width = image->getWidth();
    
    m_map = new T* [m_height];
    for(unsigned int i = 0; i < m_height; i++)
    {
        m_map[i] = new T[m_width];
    } 
}

template <class T>
void ImageMap<T>::initImageMap(Image<float>* image)
{
    Pixel<float>* pixel;
    for(unsigned int i = 0; i < m_height; i++)
    {
        for(unsigned int j = 0; j < m_width; j++)
        {
            pixel = image->getPixel(i, j);            
            m_map[i][j] = (pixel->r + pixel->g + pixel->b) / 3;
        }
    }
}

template <class T>
T** ImageMap<T>::getMapMatrix() const
{
    return m_map;
}

template <class T>
unsigned int ImageMap<T>::getHeight() const
{
    return m_height;
}

template <class T>
unsigned int ImageMap<T>::getWidth() const
{
    return m_width;
}

template <class T>
T ImageMap<T>::getValueAt(unsigned int y, unsigned int x)
{
    if(y >= 0 && y < m_height && x >= 0 && x < m_width)
    {
        return m_map[y][x];
    }
    else 
    {        
        std::cout << "Y = " << y << " X = " << x << " Height = " << m_height << " Width = " << m_width << std::endl;
        throw std::runtime_error("ImageMap:getValueAt -> Invalid Index");
    }
}

template <class T>
void ImageMap<T>::setValueAt(unsigned int y, unsigned int x, T value)
{
    if(y >= 0 && y < m_height && x >= 0 && x < m_width)
    {
        m_map[y][x] = value;
    }
    else 
    {        
        throw std::runtime_error("ImageMap:setValueAt -> Invalid Index");
    }
}


#endif	/* IMAGEMAP_H */

