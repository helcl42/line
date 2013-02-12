/* 
 * File:   BmpImage.h
 * Author: lubos
 *
 * Created on January 21, 2013, 6:51 AM
 */

#ifndef BMPIMAGE_H
#define	BMPIMAGE_H

#include <stdexcept>

#include "ros/ros.h"
#include <sensor_msgs/Image.h>

#include "PixelRGB.h"
#include "PixelLUV.h"
#include "PixelXYZ.h"

template <class T> class BmpImage;
template <class T> std::ostream& operator<<(std::ostream& out, const BmpImage<T>& img);

template <class T>
class BmpImage
{
private:
    Pixel<T>*** m_imageMatrix;

    unsigned int m_width;

    unsigned int m_height;

    unsigned int m_shrinkRatio;

public:

    BmpImage();

    BmpImage(unsigned int w, unsigned int h, unsigned int shrink = 1);

    BmpImage(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink = 1);

    virtual ~BmpImage();

    inline long getRowBytes() const;

    unsigned int getWidth() const;

    void setWidth(unsigned int width);

    unsigned int getHeight() const;

    void setHeight(unsigned int height);

    void setImageMatrix(Pixel<T>*** matrix);

    void setInstance(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink = 1);

    Pixel<T>*** getImageMatrix() const;

    void cleanUp();

    void shrinkImage(unsigned int times);

    inline Pixel<T>* getPixel(unsigned int x, unsigned int y) const;

    inline T getPixelChannelValue(unsigned int y, unsigned int x, unsigned int channel) const;

    inline void setPixel(unsigned int y, unsigned int x, Pixel<T>* pixel);

    inline void setPixelValue(unsigned int y, unsigned int x, Pixel<T>* pixel);

    inline void setPixelValue(unsigned int y, unsigned int x, T r, T g, T b);

    inline void writeToMessage(const sensor_msgs::Image::ConstPtr& img);

    friend std::ostream& operator<<<T>(std::ostream& out, const BmpImage<T>& img);
};

template <class T>
BmpImage<T>::BmpImage()
: m_imageMatrix(NULL), m_width(0), m_height(0), m_shrinkRatio(1)
{
}

template <class T>
BmpImage<T>::BmpImage(unsigned int w, unsigned int h, unsigned int shrink)
: m_shrinkRatio(shrink)
{
    m_width = w / shrink;
    m_height = h / shrink;

    m_imageMatrix = new Pixel<T>** [m_height];
    for (unsigned int i = 0; i < m_height; ++i)
    {
        m_imageMatrix[i] = new Pixel<T>* [m_width];
    }
}

template <class T>
BmpImage<T>::BmpImage(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink)
{
    if (img->width > 0 && img->height > 0)
    {
        m_width = img->width / shrink;
        m_height = img->height / shrink;

        m_imageMatrix = new Pixel<T>** [m_height];
        for (unsigned int i = 0; i < m_height; ++i)
        {
            m_imageMatrix[i] = new Pixel<T>* [m_width];
        }

        std::vector<unsigned char> data = img->data;

        int index = 0;
        Pixel<T>* pixel = NULL;
        for (int i = m_height - 1; i >= 0; --i)
        {
            for (int j = m_width - 1; j >= 0; --j, index += 3)
            {
                pixel = new PixelRGB<T > ();
                pixel->r = data[index];
                pixel->g = data[index + 1];
                pixel->b = data[index + 2];
                m_imageMatrix[i][j] = pixel;
            }
        }
    }
}

template <class T>
BmpImage<T>::~BmpImage()
{
    cleanUp();
    //    for (unsigned int i = 0; i < m_height; i++)
    //    {
    //        for (unsigned int j = 0; j < m_width; j++)
    //        {
    //            SAFE_DELETE(m_imageMatrix[i][j]);
    //        }
    //        SAFE_DELETE_ARRAY(m_imageMatrix[i]);
    //    }
    //    SAFE_DELETE_ARRAY(m_imageMatrix);
}

template <class T>
void BmpImage<T>::cleanUp()
{
    for (unsigned int i = 0; i < m_height; i++)
    {
        for (unsigned int j = 0; j < m_width; j++)
        {
            SAFE_DELETE(m_imageMatrix[i][j]);
        }
        SAFE_DELETE_ARRAY(m_imageMatrix[i]);
    }
    SAFE_DELETE_ARRAY(m_imageMatrix);

    m_imageMatrix = NULL;
    m_height = 0;
    m_width = 0;
}

template <class T>
void BmpImage<T>::writeToMessage(const sensor_msgs::Image::ConstPtr& img)
{
    if (img->width > 0 && img->height > 0)
    {
        //        short* widthPtr = (short*)img->width;
        //        *widthPtr = m_width;
        //        short* heightPtr = (short*)img->height;
        //        *heightPtr = m_height;
                
        long countOfBytes = getRowBytes();                
        
        Pixel<T>* pixel = NULL;
        unsigned char* temp;
        for (int i = 0, index = 0; i < m_height; i++)
        {            
            for (int j = 0; j < m_width; j++, index += 3)
            {
                pixel = m_imageMatrix[i][j];
                temp = (unsigned char*) &img->data[index];
                *temp = (unsigned char) pixel->b;
                temp = (unsigned char*) &img->data[index + 1];
                *temp = (unsigned char) pixel->g;
                temp = (unsigned char*) &img->data[index + 2];
                *temp = (unsigned char) pixel->r;
            }            
            index = i * img->width * 3;
        }
    }
}

template <class T>
void BmpImage<T>::setInstance(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink)
{
    if (img->width > 0 && img->height > 0)
    {
        if (m_height != img->height / shrink || m_width != img->width / shrink)
        {
            if(m_imageMatrix != NULL) 
            {
                cleanUp();
            }
            m_shrinkRatio = shrink;
            m_width = img->width / shrink;
            m_height = img->height / shrink;

            m_imageMatrix = new Pixel<T>** [m_height];
            for (unsigned int i = 0; i < m_height; ++i)
            {
                m_imageMatrix[i] = new Pixel<T>* [m_width];
            }
        }

        std::vector<unsigned char> data = img->data;

        Pixel<T>* pixel = NULL;
        for (int i = m_height - 1, index = 0; i >= 0; --i)
        {
            for (int j = m_width - 1; j >= 0; --j, index += 3 * m_shrinkRatio)
            {
                pixel = new PixelRGB<T > ();
                pixel->r = data[index];
                pixel->g = data[index + 1];
                pixel->b = data[index + 2];
                m_imageMatrix[i][j] = pixel;
            }
        }
    }
}

template <class T>
long BmpImage<T>::getRowBytes() const
{
    return ((3 * m_width + 3) >> 2) << 2;
}

template <class T>
unsigned int BmpImage<T>::getWidth() const
{
    return m_width;
}

template <class T>
void BmpImage<T>::setWidth(unsigned int width)
{
    m_width = width;
}

template <class T>
unsigned int BmpImage<T>::getHeight() const
{
    return m_height;
}

template <class T>
void BmpImage<T>::setHeight(unsigned int height)
{
    m_height = height;
}

template <class T>
void BmpImage<T>::setImageMatrix(Pixel<T>*** matrix)
{
    m_imageMatrix = matrix;
}

template <class T>
Pixel<T>*** BmpImage<T>::getImageMatrix() const
{
    return m_imageMatrix;
}

template <class T>
void BmpImage<T>::shrinkImage(unsigned int times)
{
    unsigned int width = m_width / times;
    unsigned int height = m_height / times;


    Pixel<T>*** matrix = new Pixel<T>** [height];
    for (unsigned int i = 0; i < height; ++i)
    {
        matrix[i] = new Pixel<T>* [width];
    }

    unsigned int i, j, in, jn;
    for (i = 0, in = 0; i < m_height && in < height; i += times, in++)
    {
        for (j = 0, jn = 0; j < m_width && jn < width; j += times, jn++)
        {
            matrix[in][jn] = new PixelRGB<T > (m_imageMatrix[i][j]);
        }
    }

    cleanUp();
    m_imageMatrix = matrix;
    m_height = height;
    m_width = width;
}

template <class T>
Pixel<T>* BmpImage<T>::getPixel(unsigned int y, unsigned int x) const
{
    Pixel<T>* ret = NULL;
    if (x < m_width && x >= 0 && y < m_height && y >= 0)
    {
        ret = m_imageMatrix[y][x];
    }
    else
    {
        throw std::runtime_error("Index out of bounds.");
    }
    return ret;
}

template <class T>
T BmpImage<T>::getPixelChannelValue(unsigned int y, unsigned int x, unsigned int channel) const
{
    if (x < m_width && x >= 0 && y < m_height && y >= 0)
    {
        if (channel < 4)
        {
            switch (channel)
            {
                case 0:
                    return m_imageMatrix[y][x]->r;
                case 1:
                    return m_imageMatrix[y][x]->g;
                case 2:
                    return m_imageMatrix[y][x]->b;
            }
        }
    }
    else
    {
        throw std::runtime_error("Index out of bounds.");
    }
    return 0.0;
}

template <class T>
void BmpImage<T>::setPixelValue(unsigned int y, unsigned int x, Pixel<T>* pixel)
{
    if (y < m_height && y >= 0 && x < m_width && x >= 0)
    {
        Pixel<T>* thePixel = getPixel(y, x);
        thePixel->r = pixel->r;
        thePixel->g = pixel->g;
        thePixel->b = pixel->b;
    }
    else
    {
        throw std::runtime_error("Index out of bounds.");
    }
}

template <class T>
void BmpImage<T>::setPixelValue(unsigned int y, unsigned int x, T r, T g, T b)
{
    if (y < m_height && y >= 0 && x < m_width && x >= 0)
    {
        Pixel<T>* thePixel = getPixel(y, x);
        thePixel->r = r;
        thePixel->g = g;
        thePixel->b = b;
    }
    else
    {
        throw std::runtime_error("Index out of bounds.");
    }
}

template <class T>
void BmpImage<T>::setPixel(unsigned int y, unsigned int x, Pixel<T>* pixel)
{
    if (y < m_height && y >= 0 && x < m_width && x >= 0)
    {
        m_imageMatrix[y][x] = pixel;
    }
    else
    {
        throw std::runtime_error("Index out of bounds.");
    }
}

template <class T>
std::ostream& operator<<(std::ostream& out, const BmpImage<T>& img)
{
    out << img.m_width << " x " << img.m_height << std::endl;
    for (unsigned int i = 0; i < img.m_height; i++)
    {
        for (unsigned int j = 0; j < img.m_width; j++)
        {
            out << *img.m_imageMatrix[i][j];
        }
        out << std::endl;
    }
    return out;
}

#endif	/* BMPIMAGE_H */

