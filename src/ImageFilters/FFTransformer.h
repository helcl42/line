/* 
 * File:   FFTTransformer.h
 * Author: lubos
 *
 * Created on March 27, 2013, 5:22 PM
 */

#ifndef FFTRANSFORMER_H
#define	FFTRANSFORMER_H

#include "Complex.h"
#include "../Utils/Utils.h"
#include "../Image/Image.h"
#include "../Image/ImageMap.h"

template <class T>
class FFTransformer
{
private:
    unsigned int m_height;

    unsigned int m_width;

    unsigned int m_originalWidth;

    unsigned int m_originalHeight;

private:    
    void swap(Complex<T>& a, Complex<T>& b);
    
    enum FFTDirection
    {
        FORWARD = 0,
        BACKWARD
    };

public:

    FFTransformer()
    : m_height(0), m_width(0) {}

    FFTransformer(unsigned int w, unsigned int h)
    : m_height(h), m_width(w) {}

    ~FFTransformer() {}

public:    
    static unsigned int computeImageSize(unsigned int imageWidth);

    Complex<T>** forwardFFT(ImageMap<T>* image);

    Image<T>* getAsImage(Complex<T>** input);

    void convolve(Complex<T>** base, Complex<T>** res);

    void add(Complex<T>** a, Complex<T>** b);

    void shift(Complex<T>** input);

    ImageMap<T>* inverseFFT(Complex<T>** output, bool shift = true);

    void inverseFFT(Complex<T>** output, ImageMap<T>* map, bool shift = true);
    
    void inverseFFTInner(Complex<T>** output, ImageMap<T>* map, bool shift);

    void FFT2D(FFTDirection direction, Complex<T>** output);

    void FFT1D(FFTDirection direction, Complex<T> x[], long m, long size);
    
public:    
    void setHeight(unsigned int height);

    unsigned int getHeight() const;

    void setWidth(unsigned int width);

    unsigned int getWidth() const;

    void setOriginalHeight(unsigned int originalHeight);

    unsigned int getOriginalHeight() const;

    unsigned int getOriginalWidth() const;

    void setOriginalWidth(unsigned int originalWidth);
};


template <class T>
unsigned int FFTransformer<T>::computeImageSize(unsigned int imageWidth)
{
    int multed = 0;
    int res = (int) Utils::logBy(imageWidth, 2);

    multed = std::pow(2, res);
    if (multed < imageWidth) res++;

    return std::pow(2, res);
}

template <class T>
Complex<T>** FFTransformer<T>::forwardFFT(ImageMap<T>* image)
{
    unsigned int lowHeight = (m_height >> 1) - (m_originalHeight >> 1);
    unsigned int lowWidth = (m_width >> 1) - (m_originalWidth >> 1);
    unsigned int hiHeight = (m_height >> 1) + (m_originalHeight >> 1);
    unsigned int hiWidth = (m_width >> 1) + (m_originalWidth >> 1);

    if (m_originalHeight % 2 != 0) hiHeight++;
    if (m_originalWidth % 2 != 0) hiWidth++;

    //std::cout << "LowHeight = " << lowHeight << " LowWidth = " << lowWidth << " hiH = " << hiHeight << " hiW = " << hiWidth << std::endl;

    Complex<T>** out = new Complex<T>*[m_height];
    for (unsigned int i = 0; i < m_height; i++)
    {
        out[i] = new Complex<T>[m_width];
        for (unsigned int j = 0; j < m_width; j++)
        {
            out[i][j].real = 0;
            out[i][j].imag = 0;
        }
    }

    for (unsigned int i = lowHeight, vk = 0; i < hiHeight; i++, vk++)
    {
        for (unsigned int j = lowWidth, hk = 0; j < hiWidth; j++, hk++)
        {
            out[i][j].real = (T) image->getValueAt(vk, hk);
            out[i][j].imag = 0;
        }
    }

    FFT2D(FORWARD, out);
    return out;
}

template <class T>
Image<T>* FFTransformer<T>::getAsImage(Complex<T>** input)
{
    Complex<T>** shifted = shift(input);

    Image<T>* image = new Image<T > (m_width, m_height);

    float value;
    for (unsigned int i = 0; i < m_height; i++)
    {
        for (unsigned int j = 0; j < m_width; j++)
        {
            value = shifted[i][j].magnitude() * 50;
            image->setPixelValue(j, i, value, value, value);
        }
    }
    return image;
}

template <class T>
void FFTransformer<T>::convolve(Complex<T>** base, Complex<T>** res)
{
    for (unsigned int i = 0; i < m_height; i++)
    {
        for (unsigned int j = 0; j < m_width; j++)
        {           
            res[i][j] = base[i][j] * res[i][j];
        }
    }
}

template <class T>
void FFTransformer<T>::add(Complex<T>** a, Complex<T>** b)
{
    for (unsigned int i = 0; i < m_height; i++)
    {
        for (unsigned int j = 0; j < m_width; j++)
        {
            a[i][j] += b[i][j];
        }
    }
}

template <class T>
void FFTransformer<T>::shift(Complex<T>** input)
{
    Complex<T> temp;
    for (unsigned int i = 0; i < (m_width >> 1); i++)
    {
        for (unsigned int j = 0; j < (m_height >> 1); j++)
        {
            temp = input[i + (m_width >> 1)][j + (m_height >> 1)];
            input[i + (m_width >> 1)][j + (m_height >> 1)] = input[i][j];
            input[i][j] = temp;
            temp = input[i + (m_width >> 1)][j];
            input[i + (m_width >> 1)][j] = input[i][j + (m_height >> 1)];
            input[i][j + (m_width >> 1)] = temp;
        }
    }
}

template <class T>
ImageMap<T>* FFTransformer<T>::inverseFFT(Complex<T>** output, bool shift)
{
    ImageMap<T>* map;

    FFT2D(BACKWARD, output);

    map = new ImageMap<T > (m_height, m_width);
    inverseFFTInner(output, map, shift);

    return map;
}

template <class T>
void FFTransformer<T>::inverseFFT(Complex<T>** output, ImageMap<T>* map, bool shift)
{
    FFT2D(BACKWARD, output);

    inverseFFTInner(output, map, shift);
}

template <class T>
void FFTransformer<T>::inverseFFTInner(Complex<T>** output, ImageMap<T>* map, bool shift)
{
    unsigned int lowHeight, lowWidth;
    unsigned int hiHeight, hiWidth;

    //std::cout << "LowHeight = " << lowHeight << " LowWidth = " << lowWidth << " hiH = " << hiHeight << " hiW = " << hiWidth << std::endl;

    if (shift)
    {
        hiHeight = m_height - (m_originalHeight >> 1);
        hiWidth = m_width - (m_originalWidth >> 1);

        for (unsigned int i = 0; i < (m_originalHeight >> 1); i++)
        {
            for (unsigned int j = 0; j < (m_originalWidth >> 1); j++)
            {
                map->setValueAt(i + (m_originalHeight >> 1), j + (m_originalWidth >> 1), Utils::normalize<int>(output[i][j].magnitude()));
                map->setValueAt(i, j, Utils::normalize<int>(output[i + hiHeight][j + hiWidth].magnitude()));
                map->setValueAt(i, j + (m_originalWidth >> 1), Utils::normalize<int>(output[i + hiHeight][j].magnitude()));
                map->setValueAt(i + (m_originalHeight >> 1), j, Utils::normalize<int>(output[i][j + hiWidth].magnitude()));
            }
        }
    }
    else
    {
        lowHeight = (m_height >> 1) - (m_originalHeight >> 1);
        lowWidth = (m_width >> 1) - (m_originalWidth >> 1);
        hiHeight = (m_height >> 1) + (m_originalHeight >> 1);
        hiWidth = (m_width >> 1) + (m_originalWidth >> 1);

        if (m_originalHeight % 2 != 0) hiHeight++;
        if (m_originalWidth % 2 != 0) hiWidth++;

        for (unsigned int i = lowHeight, vk = 0; i < hiHeight; i++, vk++)
        {
            for (unsigned int j = lowWidth, hk = 0; j < hiWidth; j++, hk++)
            {
                map->setValueAt(vk, hk, Utils::normalize<int>(output[i][j].magnitude()));
            }
        }
    }
}

template <class T>
void FFTransformer<T>::FFT2D(FFTDirection direction, Complex<T>** output)
{
    int i, j;
    int m = (int) Utils::logBy(m_width, 2);

    Complex<T>* data = new Complex<T>[m_width];

    for (j = 0; j < m_height; j++)
    {
        for (i = 0; i < m_width; i++)
        {
            data[i] = output[i][j];
        }

        FFT1D(direction, data, m, m_width);

        for (i = 0; i < m_width; i++)
        {
            output[i][j] = data[i];
        }
    }

    for (i = 0; i < m_width; i++)
    {
        for (j = 0; j < m_height; j++)
        {
            data[j] = output[i][j];
        }

        FFT1D(direction, data, m, m_width);

        for (j = 0; j < m_height; j++)
        {
            output[i][j] = data[j];
        }
    }

    SAFE_DELETE_ARRAY(data);
}

template <class T>
void FFTransformer<T>::FFT1D(FFTDirection direction, Complex<T> x[], long m, long size)
{
    long i, i1, i2, j = 0, k, l, l1, l2 = 1;
    Complex<T> t1, u, c;

    i2 = size >> 1;

    for (i = 0; i < size - 1; i++)
    {
        if (i < j) swap(x[i], x[j]);

        k = i2;

        while (k <= j)
        {
            j -= k;
            k >>= 1;
        }

        j += k;
    }

    c.real = -1.0;
    c.imag = 0.0;

    for (l = 0; l < m; l++)
    {
        l1 = l2;
        l2 <<= 1;
        u.real = 1.0;
        u.imag = 0.0;

        for (j = 0; j < l1; j++)
        {
            for (i = j; i < size; i += l2)
            {
                i1 = i + l1;
                t1 = u * x[i1];
                x[i1] = x[i] - t1;
                x[i] += t1;
            }

            u = u * c;
        }

        c.imag = std::sqrt((1.0 - c.real) / 2.0);

        if (direction == FORWARD) c.imag = -c.imag;

        c.real = std::sqrt((1.0 + c.real) / 2.0);
    }

    if (direction == BACKWARD)
    {
        for (i = 0; i < size; i++) x[i] /= size;
    }
}

template <class T>
void FFTransformer<T>::swap(Complex<T>& a, Complex<T>& b)
{
    Complex<T> temp;
    temp = a;
    a = b;
    b = temp;
}

template <class T>
void FFTransformer<T>::setHeight(unsigned int height)
{
    m_height = height;
}

template <class T>
unsigned int FFTransformer<T>::getHeight() const
{
    return m_height;
}

template <class T>
void FFTransformer<T>::setWidth(unsigned int width)
{
    m_width = width;
}

template <class T>
unsigned int FFTransformer<T>::getWidth() const
{
    return m_width;
}

template <class T>
void FFTransformer<T>::setOriginalHeight(unsigned int originalHeight)
{
    this->m_originalHeight = originalHeight;
}

template <class T>
unsigned int FFTransformer<T>::getOriginalHeight() const
{
    return m_originalHeight;
}

template <class T>
unsigned int FFTransformer<T>::getOriginalWidth() const
{
    return m_originalWidth;
}

template <class T>
void FFTransformer<T>::setOriginalWidth(unsigned int originalWidth)
{
    this->m_originalWidth = originalWidth;
}

#endif	/* FFTRANSFORMER_H */


