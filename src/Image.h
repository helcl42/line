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

#include "Pixel/PixelRGB.h"
#include "Pixel/PixelLUV.h"
#include "Pixel/PixelLUV.h"
#include "Line.h"

template <class T> class Image;
template <class T> std::ostream& operator<<(std::ostream& out, const Image<T>& img);

template <class T>
class Image
{
private:
    Pixel<T>*** m_imageMatrix;

    unsigned int m_width;

    unsigned int m_height;

    unsigned int m_shrinkRatio;

public:

    Image();

    Image(unsigned int w, unsigned int h, unsigned int shrink = 1);

    Image(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink = 1);

    virtual ~Image();
        
    void setInstance(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink = 1);

    inline long getRowBytes() const;

    unsigned int getWidth() const;

    void setWidth(unsigned int width);

    unsigned int getHeight() const;

    void setHeight(unsigned int height);

    void setImageMatrix(Pixel<T>*** matrix);

    Pixel<T>*** getImageMatrix() const;

    void cleanUp();

    void shrinkImage(unsigned int times);
    
    //temp
    void writeCircle(int x0, int y0, int radius, double angle);
    //temp
    void writeSquare(int x0, int y0, int height, int width, double angle, double rotateAngle, double anotherAngle);

    inline Pixel<T>* getPixel(unsigned int x, unsigned int y) const;

    inline T getPixelChannelValue(unsigned int y, unsigned int x, unsigned int channel) const;

    inline void setPixel(unsigned int y, unsigned int x, Pixel<T>* pixel);

    inline void setPixelValue(unsigned int y, unsigned int x, Pixel<T>* pixel);

    inline void setPixelValue(unsigned int y, unsigned int x, T r, T g, T b);

    //inline void writeToMessage(const sensor_msgs::Image::ConstPtr& img);        

    friend std::ostream& operator<<<T>(std::ostream& out, const Image<T>& img);
};

template <class T>
Image<T>::Image()
: m_imageMatrix(NULL), m_width(0), m_height(0), m_shrinkRatio(1)
{
}

template <class T>
Image<T>::Image(unsigned int w, unsigned int h, unsigned int shrink)
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
Image<T>::Image(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink)
{
    setInstance(img, shrink);
}

template <class T>
Image<T>::~Image()
{
    cleanUp();    
}

template <class T>
void Image<T>::cleanUp()
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

//template <class T>
//void BmpImage<T>::writeToMessage(const sensor_msgs::Image::ConstPtr& img)
//{
//    if (img->width > 0 && img->height > 0)
//    {        
//        Pixel<T>* pixel = NULL;
//        unsigned char* temp;
//        for (int i = 0, index = 0; i < m_height; i++)
//        {            
//            for (int j = 0; j < m_width; j++, index += 3)
//            {
//                pixel = m_imageMatrix[i][j];
//                temp = (unsigned char*) &img->data[index];
//                *temp = (unsigned char) pixel->b;
//                temp = (unsigned char*) &img->data[index + 1];
//                *temp = (unsigned char) pixel->g;
//                temp = (unsigned char*) &img->data[index + 2];
//                *temp = (unsigned char) pixel->r;
//            }            
//            index = i * img->width * 3;
//        }
//    }
//}

template <class T>
void Image<T>::setInstance(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink)
{
    unsigned int corrector = 0;
    if (img->width > 0 && img->height > 0)
    {                
        //allocate first time only
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
                for (unsigned int j = 0; j < m_width; ++j) 
                {
                    m_imageMatrix[i][j] = new PixelRGB<T>(0, 0, 0);
                }
            }
        }       

        corrector = (img->width % m_shrinkRatio) * m_shrinkRatio;
        
        std::vector<unsigned char> data = img->data;

        Pixel<T>* pixel = NULL;
        for (int i = m_height - 1, index = 0; i >= 0; --i)
        {
            for (int j = m_width - 1; j >= 0; --j, index += 3 * m_shrinkRatio)
            {
                pixel = m_imageMatrix[i][j];
                pixel->r = data[index];
                pixel->g = data[index + 1];
                pixel->b = data[index + 2];
                m_imageMatrix[i][j] = pixel;
            }
            index += 3 * img->width * (m_shrinkRatio - 1) + corrector;
        }
    }
}

template <class T>
void Image<T>::writeSquare(int x0, int y0, int height, int width, double angle, double rotateAngle, double anotherAngle)
{
    Line* square = new Line();
    //height = height * sin(angle * M_PI / 180);
//    for(int i = y0; i < y0 + height; i++)
//    {
//        for(int j = x0; j < x0 + width; j++)
//        {            
//            if(i == y0 || i == y0 + height - 1) 
//            {
//                setPixelValue((j * sin(rotateAngle * M_PI / 180) + i * cos(rotateAngle * M_PI / 180)) * sin(angle * M_PI / 180), j * cos(rotateAngle * M_PI / 180) - i * sin(rotateAngle * M_PI / 180) * cos(anotherAngle * M_PI / 180), 0, 0, 255);
//            }
//            else 
//            {
//                if(j == x0 || j == x0 + width - 1)
//                {
//                    setPixelValue((j * sin(rotateAngle * M_PI / 180) + i * cos(rotateAngle * M_PI / 180)) * sin(angle * M_PI / 180), j * cos(rotateAngle * M_PI / 180) - i * sin(rotateAngle * M_PI / 180) * cos(anotherAngle * M_PI / 180), 0, 0, 255);
//                }                
//            }
//        }
//    }
        
    double sinAngle = sin(rotateAngle * M_PI / 180);
    double cosAngle = cos(rotateAngle * M_PI / 180);   
    int halfSize = height;
    
    for (int i = y0 - halfSize; i < y0 + halfSize; i++)
    {
        for (int j = x0 - halfSize; j < x0 + halfSize; j++)
        {
            if (i == y0 - halfSize || i == y0 + halfSize - 1)
            {                              
                square->points.push_back(Vector2<int>(((j - x0) * cosAngle - (i - y0) * sinAngle) * sin(angle * M_PI / 180), ((j - x0) * sinAngle + (i - y0) * cosAngle)));                                
            }
            else
            {
                if (j == x0 - halfSize || j == x0 + halfSize - 1)
                {                    
                    square->points.push_back(Vector2<int>(((j - x0) * cosAngle - (i - y0) * sinAngle) * sin(angle * M_PI / 180), ((j - x0) * sinAngle + (i - y0) * cosAngle)));                 
                }
            }
        }
    }
    
    for(int i = 0; i < square->getSize(); i++)
    {        
        square->points[i] = Vector2<int>(square->points[i].x + x0 + halfSize, square->points[i].y + y0 + halfSize);
        setPixelValue(square->points[i].y, square->points[i].x, 0, 0, 255);
    }  
    SAFE_DELETE(square);    
}

template <class T>
void Image<T>::writeCircle(int x0, int y0, int radius, double angle)
{
    int f = 1 - radius;
    int ddFx = 1;
    int ddFy = -2 * radius;// * cos(angle * M_PI / 180);
    int x = 0;
    int y = radius; // * cos(angle * M_PI / 180);
    float ellipse = sin(angle * M_PI / 180);
 
    setPixelValue(y0 + radius * ellipse, x0, 0, 0, 255);
    setPixelValue(y0 - radius * ellipse, x0, 0, 0, 255);
    setPixelValue(y0, x0 + radius, 0, 0, 255);
    setPixelValue(y0, x0 - radius, 0, 0, 255);
 
    while(x < y)
    {
        // ddF_x == 2 * x + 1;
        // ddF_y == -2 * y;
        // f == x*x + y*y - radius*radius + 2*x - y + 1;
        if(f >= 0) 
        {
            y--;
            ddFy += 2;
            f += ddFy;
        }
        
        x++;
        ddFx += 2;
        f += ddFx;    
        
        setPixelValue(y0 + y * ellipse, x0 + x, 0, 0, 255);
        setPixelValue(y0 + y * ellipse, x0 - x, 0, 0, 255);
        setPixelValue(y0 - y * ellipse, x0 + x, 0, 0, 255);
        setPixelValue(y0 - y * ellipse, x0 - x, 0, 0, 255);
        
        setPixelValue(y0 + x * ellipse, x0 + y, 0, 0, 255);
        setPixelValue(y0 + x * ellipse, x0 - y, 0, 0, 255);
        setPixelValue(y0 - x * ellipse, x0 + y, 0, 0, 255);
        setPixelValue(y0 - x * ellipse, x0 - y, 0, 0, 255);
    }
}

template <class T>
long Image<T>::getRowBytes() const
{
    return ((3 * m_width + 3) >> 2) << 2;
}

template <class T>
unsigned int Image<T>::getWidth() const
{
    return m_width;
}

template <class T>
void Image<T>::setWidth(unsigned int width)
{
    m_width = width;
}

template <class T>
unsigned int Image<T>::getHeight() const
{
    return m_height;
}

template <class T>
void Image<T>::setHeight(unsigned int height)
{
    m_height = height;
}

template <class T>
void Image<T>::setImageMatrix(Pixel<T>*** matrix)
{
    m_imageMatrix = matrix;
}

template <class T>
Pixel<T>*** Image<T>::getImageMatrix() const
{
    return m_imageMatrix;
}

template <class T>
void Image<T>::shrinkImage(unsigned int times)
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
Pixel<T>* Image<T>::getPixel(unsigned int y, unsigned int x) const
{
    Pixel<T>* ret = NULL;
    if (x < m_width && x >= 0 && y < m_height && y >= 0)
    {
        ret = m_imageMatrix[y][x];
    }
    else
    {
        throw std::runtime_error("Image:getPixel ->Index out of bounds.");
    }
    return ret;
}

template <class T>
T Image<T>::getPixelChannelValue(unsigned int y, unsigned int x, unsigned int channel) const
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
        throw std::runtime_error("Image:GetPixelChannelValue ->Index out of bounds.");
    }
    return 0.0;
}

template <class T>
void Image<T>::setPixelValue(unsigned int y, unsigned int x, Pixel<T>* pixel)
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
        throw std::runtime_error("Image:SetPixelValue -> Index out of bounds.");
    }
}

template <class T>
void Image<T>::setPixelValue(unsigned int y, unsigned int x, T r, T g, T b)
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
        std::cout << "y = " << y << " x = " << x << std::endl;
        throw std::runtime_error("Image:SetPixelValue -> Index out of bounds.");
    }
}

template <class T>
void Image<T>::setPixel(unsigned int y, unsigned int x, Pixel<T>* pixel)
{
    if (y < m_height && y >= 0 && x < m_width && x >= 0)
    {
        m_imageMatrix[y][x] = pixel;
    }
    else
    {
        throw std::runtime_error("Image:SetPixel ->Index out of bounds.");
    }
}

template <class T>
std::ostream& operator<<(std::ostream& out, const Image<T>& img)
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

