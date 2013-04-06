/* 
 * File:   ImageFilter.h
 * Author: lubos
 *
 * Created on March 31, 2013, 1:08 AM
 */

#ifndef IMAGEFILTER_H
#define	IMAGEFILTER_H

#include "../../Image/Image.h"
#include "../Kernel/Kernel.h"
#include "../IImageFilter.h"

template <class T>
class ImageFilter : public IImageFilter<T>
{
protected:
    Image<T>* m_image;

    Kernel<T>* m_kernel;

public:

    ImageFilter()
    : m_image(NULL), m_kernel(NULL) {}

    ImageFilter(Kernel<T>* kernel)
    : m_image(NULL), m_kernel(kernel) {}

    ImageFilter(Image<T>* image, Kernel<T>* kernel)
    : m_image(image), m_kernel(kernel) {}

    virtual ~ImageFilter() {}

public:
    void setKernel(Kernel<T>* kernel);

    Kernel<T>* getKernel() const;

    void setImage(Image<T>* image);

    Image<T>* getImage() const;

public:
    inline double getValueForChannel(unsigned int y, unsigned int x, int ch);

    void applyFilter();
};

template <class T>
void ImageFilter<T>::setImage(Image<T>* image)
{
    m_image = image;
}

template <class T>
void ImageFilter<T>::setKernel(Kernel<T>* kernel)
{
    m_kernel = kernel;
}

template <class T>
Kernel<T>* ImageFilter<T>::getKernel() const
{
    return m_kernel;
}

template <class T>
Image<T>* ImageFilter<T>::getImage() const
{
    return m_image;
}

template <class T>
inline double ImageFilter<T>::getValueForChannel(unsigned int y, unsigned int x, int ch)
{
    double result = 0;    

    unsigned int heightHalf = (this->m_kernel->getHeight() >> 1) <= 0 ? 1 : this->m_kernel->getHeight() >> 1;
    unsigned int widthHalf = (this->m_kernel->getWidth() >> 1) <= 0 ? 1 : this->m_kernel->getWidth() >> 1;

    if (this->m_kernel->isSeparable())
    {
        for (unsigned int k = 0; k < 2; k++)
        {
            Kernel<T>& tempKernel = this->m_kernel[k];
            for (unsigned int i = y - heightHalf, hk = 0; i < y + heightHalf + 1; i++, hk++)
            {
                for (unsigned int j = x - widthHalf, wk = 0; j < x + widthHalf + 1; j++, wk++)
                {
                    result += this->m_image->getPixelChannelValue(i, j, ch) * tempKernel.getValue(hk, wk);
                }
            }
        }
    }
    else
    {
        for (unsigned int i = y - heightHalf, hk = 0; i < y + heightHalf + 1; i++, hk++)
        {
            for (unsigned int j = x - widthHalf, wk = 0; j < x + widthHalf + 1; j++, wk++)
            {
                result += this->m_image->getPixelChannelValue(i, j, ch) * this->m_kernel->getValue(hk, wk);
            }
        }
    }
    return Utils::normalize<T > (result);
}

template <class T>
void ImageFilter<T>::applyFilter()
{
    double channelResult;
    Pixel<float>* pixel = NULL;

    unsigned int kerHeightHalf = (this->m_kernel->getHeight() >> 1) <= 0 ? 1 : this->m_kernel->getHeight() >> 1;
    unsigned int kerWidthHalf = (this->m_kernel->getWidth() >> 1) <= 0 ? 1 : this->m_kernel->getWidth() >> 1;

    for (unsigned int y = kerHeightHalf; y < this->m_image->getHeight() - kerHeightHalf; y++)
    {
        for (unsigned int x = kerWidthHalf; x < this->m_image->getWidth() - kerWidthHalf; x++)
        {
            pixel = this->m_image->getPixel(y, x);

            for (int ch = 0; ch < 3; ch++)
            {
                channelResult = getValueForChannel(y, x, ch);

                switch (ch)
                {
                    case 0:
                        pixel->r = channelResult;
                        break;
                    case 1:
                        pixel->g = channelResult;
                        break;
                    case 2:
                        pixel->b = channelResult;
                        break;
                }
            }
            this->m_image->setPixelValue(y, x, pixel);
        }
    }
}

#endif	/* IMAGEFILTER_H */

