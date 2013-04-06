/* 
 * File:   ImageMap.h
 * Author: lubos
 *
 * Created on March 27, 2013, 5:39 PM
 */

#ifndef IMAGEMAP_H
#define	IMAGEMAP_H

#include "Image.h"
#include "../ImageFilters/Kernel/Kernel.h"

template <class T> class ImageMap;
template <class T> std::ostream& operator<<(std::ostream& out, const ImageMap<T>& imageMap);

template <class T>
class ImageMap
{
private:
    T** m_map;

    unsigned int m_width;

    unsigned int m_height;

public:
    ImageMap();

    ImageMap(Image<T>* image);

    ImageMap(unsigned int h, unsigned int w, Kernel<T>* kernel);

    ImageMap(unsigned int h, unsigned int w);

    ~ImageMap();

    T** getMapMatrix() const;

    T getValueAt(unsigned int y, unsigned int x);

    void setValueAt(unsigned int y, unsigned int x, T value);

    unsigned int getWidth() const;

    unsigned int getHeight() const;

    void setInstance(Image<T>* image, unsigned int shrink);

    void setInstance(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink);
    
    void resolveThreshold(float threshold = 50);

    Image<T>* getAsImage();

    friend std::ostream& operator<<<T>(std::ostream& out, const ImageMap<T>& imageMap);

protected:
    void initImageMap(Image<T>* image, unsigned int shrink);

private:
    void cleanUp();

    void allocateMap(Image<T>* image, unsigned int shrink);

    void allocateMap(unsigned int h, unsigned int w);
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
    for (unsigned int i = 0; i < m_height; i++)
    {
        m_map[i] = new T[m_width];
    }
}

template <class T>
ImageMap<T>::ImageMap(unsigned int h, unsigned int w, Kernel<T>* kernel)
: m_height(h), m_width(w)
{
    unsigned int heightHalf = (kernel->getHeight() >> 1) <= 0 ? 1 : kernel->getHeight() >> 1;
    unsigned int widthHalf = (kernel->getWidth() >> 1) <= 0 ? 1 : kernel->getWidth() >> 1;

    m_map = new T* [m_height];
    for (unsigned int i = 0; i < m_height; i++)
    {
        m_map[i] = new T[m_width];
        for (unsigned int j = 0; j < m_width; j++)
        {
            m_map[i][j] = 0;
        }
    }

    for (unsigned int i = (m_height >> 1) - heightHalf, vk = 0; i < (m_height >> 1) + heightHalf + 1; i++, vk++)
    {
        for (unsigned int j = (m_width >> 1) - widthHalf, hk = 0; j < (m_width >> 1) + widthHalf + 1; j++, hk++)
        {
            m_map[i][j] = kernel->getValue(vk, hk);
        }
    }
}

template <class T>
ImageMap<T>::ImageMap(Image<T>* image)
: m_width(image->getWidth()), m_height(image->getHeight())
{
    m_map = new T* [m_height];
    for (unsigned int i = 0; i < m_height; i++)
    {
        m_map[i] = new T[m_width];
    }
    initImageMap(image, 1);
}

template <class T>
void ImageMap<T>::cleanUp()
{
    for (unsigned int i = 0; i < m_height; i++)
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
void ImageMap<T>::resolveThreshold(float threshold)
{
    for (unsigned int i = 0; i < m_height; i++)
    {
        for (unsigned int j = 0; j < m_width; j++)
        {
            m_map[i][j] = Utils::colorThreshold(m_map[i][j], threshold);
        }
    }
}

template <class T>
Image<T>* ImageMap<T>::getAsImage()
{
    Image<T>* image = new Image<T > (m_width, m_height);

    T value;
    for (unsigned int i = 0; i < m_height; i++)
    {
        for (unsigned int j = 0; j < m_width; j++)
        {
            value = m_map[i][j];
            image->setPixelValue(i, j, value, value, value);
        }
    }
    return image;
}

template <class T>
void ImageMap<T>::setInstance(Image<T>* image, unsigned int shrink)
{
    if (m_map == NULL)
    {
        allocateMap(image, shrink);
    }
    else if (m_height < image->getHeight() / shrink || m_width < image->getWidth() / shrink)
    {
        cleanUp();
        allocateMap(image, shrink);
    }

    initImageMap(image);
}

template <class T>
void ImageMap<T>::setInstance(const sensor_msgs::Image::ConstPtr& img, unsigned int shrink)
{
    unsigned int corrector = 0;
    if (m_height != img->height / shrink || m_width != img->width / shrink)
    {
        m_height = img->height / shrink;
        m_width = img->width / shrink;
        
        allocateMap(m_height, m_width);
    }

    corrector = (img->width % shrink) * 3;

    std::vector<unsigned char> data = img->data;

    T value;
    for (int i = m_height - 1, index = 0; i >= 0; --i)
    {
        for (int j = m_width - 1; j >= 0; --j, index += 3 * shrink)
        {            
            m_map[i][j] = (data[index] + data[index + 1] + data[index + 2]) / 3;                                    
        }
        index += 3 * img->width * (shrink - 1) + corrector;
    }
}

template <class T>
void ImageMap<T>::allocateMap(Image<T>* image, unsigned int shrink)
{
    m_height = image->getHeight() / shrink;
    m_width = image->getWidth() / shrink;

    m_map = new T* [m_height];
    for (unsigned int i = 0; i < m_height; i++)
    {
        m_map[i] = new T[m_width];
    }
}

template <class T>
void ImageMap<T>::allocateMap(unsigned int h, unsigned int w)
{
    m_map = new T* [h];
    for (unsigned int i = 0; i < h; i++)
    {
        m_map[i] = new T[w];
    }
}

template <class T>
void ImageMap<T>::initImageMap(Image<T>* image, unsigned int shrink)
{
    Pixel<T>* pixel;
    for (unsigned int i = 0; i < m_height; i += shrink)
    {
        for (unsigned int j = 0; j < m_width; j += shrink)
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
    if (y >= 0 && y < m_height && x >= 0 && x < m_width)
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
    if (y >= 0 && y < m_height && x >= 0 && x < m_width)
    {
        m_map[y][x] = value;
    }
    else
    {
        std::cout << "FAIL: y = " << y << ", x = " << x << std::endl;
        throw std::runtime_error("ImageMap:setValueAt -> Invalid Index");
    }
}

template <class T>
std::ostream& operator<<(std::ostream& out, const ImageMap<T>& imageMap)
{
    for (unsigned int i = 0; i < imageMap.m_height; i++)
    {
        for (unsigned int j = 0; j < imageMap.m_width; j++)
        {
            out << imageMap.m_map[i][j] << " ";
        }
        out << std::endl;
    }
    return out;
}

#endif	/* IMAGEMAP_H */

