/* 
 * File:   AbstractFFTImageFilter.h
 * Author: lubos
 *
 * Created on April 5, 2013, 5:36 AM
 */

#ifndef ABSTRACTFFTIMAGEFILTER_H
#define	ABSTRACTFFTIMAGEFILTER_H

#include "../Complex.h"
#include "../FFTransformer.h"
#include "../IImageFilter.h"
#include "../../Image/ImageMap.h"

template <class T>
class AbstractFFTImageFilter : public IImageFilter<T>
{
protected:
    unsigned int m_width;

    unsigned int m_height;

    unsigned int m_originalWidth;

    unsigned int m_originalHeight;

    ImageMap<T>* m_imageMap;

    Complex<T>** m_fftState;

    FFTransformer<T>* m_fft;

public:

    AbstractFFTImageFilter()
    :  m_width(0), m_height(0), m_originalWidth(0), m_originalHeight(0), m_imageMap(NULL), m_fftState(NULL)
    {        
        m_fft = new FFTransformer<T > ();
    }
    
    
    AbstractFFTImageFilter(ImageMap<T>* image)
    : m_height(image->getHeight()), m_width(image->getWidth()), m_imageMap(image)
    {
        unsigned int size;

        m_fft = new FFTransformer<T > ();
        
        m_originalHeight = image->getHeight();
        m_originalWidth = image->getWidth();

        size = m_originalWidth > m_originalHeight ? m_originalWidth : m_originalHeight;
        m_width = m_height = FFTransformer<T>::computeImageSize(size);
        
        m_fft->setHeight(m_height);
        m_fft->setWidth(m_width);
        m_fft->setOriginalHeight(m_originalHeight);
        m_fft->setOriginalWidth(m_originalWidth);
    }
    

    virtual ~AbstractFFTImageFilter()
    {
        for (unsigned int i = 0; i < m_height; i++)
        {
            SAFE_DELETE_ARRAY(m_fftState[i])
        }
        SAFE_DELETE_ARRAY(m_fftState);
        
        SAFE_DELETE(m_fft);
    }

public:

    FFTransformer<T>* getFft() const
    {
        return m_fft;
    }

    void setFft(FFTransformer<T>* fft)
    {
        this->m_fft = fft;
    }

    Complex<T>** getFftState() const
    {
        return m_fftState;
    }

    void setFftState(Complex<T>** fftState)
    {
        this->m_fftState = fftState;
    }

    unsigned int getHeight() const
    {
        return m_height;
    }

    void setHeight(unsigned int height)
    {
        this->m_height = height;
    }

    ImageMap<T>* getImageMap() const
    {
        return m_imageMap;
    }

    void setImageMap(ImageMap<T>* imageMap)
    {
        this->m_imageMap = imageMap;
    }

    unsigned int getWidth() const
    {
        return m_width;
    }

    void setWidth(unsigned int width)
    {
        this->m_width = width;
    }
};

#endif	/* ABSTRACTFFTIMAGEFILTER_H */

