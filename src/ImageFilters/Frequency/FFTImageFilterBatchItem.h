/* 
 * File:   FFTImageFilterBatchItem.h
 * Author: lubos
 *
 * Created on April 3, 2013, 11:52 PM
 */

#ifndef FFTIMAGEFILTERBATCHITEM_H
#define	FFTIMAGEFILTERBATCHITEM_H

#include "../Complex.h"
#include "../Kernel/Kernel.h"
#include "../Kernel/NonSeparableKernel.h"
#include "../FFTransformer.h"

template <class T>
class FFTImageFilterBatchItem
{
private:
    unsigned int m_width;

    unsigned int m_height;

    ImageMap<T>* m_imageMap;

    std::vector<Complex<T>**> m_kernelStates;

    std::vector<Kernel<T>* > m_kernels;

    FFTransformer<T>* m_fft;

public:

    FFTImageFilterBatchItem(Kernel<T>* kernelX)
    : m_height(0), m_width(0), m_imageMap(NULL), m_fft(NULL)
    {
        m_kernels.push_back(kernelX);
    }

    FFTImageFilterBatchItem(std::vector<Kernel<T>*>& kernels)
    : m_height(0), m_width(0), m_imageMap(NULL), m_fft(NULL)
    {
        typename std::vector<Kernel<T>* >::iterator ii;
        for (ii = kernels.begin(); ii != kernels.end(); ++ii)
        {
            m_kernels.push_back(*ii);
        }
    }

    virtual ~FFTImageFilterBatchItem()
    {
        typename std::vector<Kernel<T> *>::iterator ii;
        for (ii = m_kernels.begin(); ii != m_kernels.end(); ++ii)
        {
            SAFE_DELETE(*ii);
        }

        cleanUp();
    }

    void cleanUp()
    {
        typename std::vector<Complex<T>**>::iterator jj;
        for (jj = m_kernelStates.begin(); jj != m_kernelStates.end(); ++jj)
        {
            for (unsigned int i = 0; i < m_height; i++)
            {
                SAFE_DELETE_ARRAY((*jj)[i]);
            }
            SAFE_DELETE_ARRAY((*jj));
        }
        m_kernelStates.clear();
    }

    void setInstance(ImageMap<float>* image, FFTransformer<T>* fftInstance)
    {        
        ImageMap<T>* imageMap = NULL;
        unsigned int size;
        unsigned int originalHeight = image->getHeight();
        unsigned int originalWidth = image->getWidth();

        m_fft = fftInstance;
                
        if (originalHeight != m_height || originalWidth != m_width)
        {
            cleanUp();
            
            size = originalWidth > originalHeight ? originalWidth : originalHeight;
            m_width = m_height = FFTransformer<float>::computeImageSize(size);

            typename std::vector<Kernel<T> *>::iterator ii;
            for (ii = m_kernels.begin(); ii != m_kernels.end(); ++ii)
            {
                imageMap = new ImageMap<T > (m_height, m_width, *ii);                                       
                m_kernelStates.push_back(m_fft->forwardFFT(imageMap));
                SAFE_DELETE(imageMap);
            }          
            
            m_fft->setOriginalHeight(originalHeight);
            m_fft->setOriginalWidth(originalWidth);
        }
    }

    Complex<T>** applyFilterItem(Complex<T>** input)
    {
        std::vector<Complex<T>**> tempResults;
        typename std::vector<Complex<T>**>::iterator ii;

        for (ii = m_kernelStates.begin(); ii != m_kernelStates.end(); ++ii)
        {            
            m_fft->convolve(input, *ii);
            tempResults.push_back(*ii);
        }

        if (tempResults.size() > 1)
        {            
            for (unsigned int i = 0; i < tempResults.size() - 1; i++)
            {
                m_fft->add(tempResults[i + 1], tempResults[i]);
            }

            for (ii = tempResults.begin(); ii != tempResults.end() - 1; ++ii)
            {
                for (unsigned int j = 0; j < m_height; j++)
                {
                    SAFE_DELETE_ARRAY((*ii)[j])
                }
            }
        }
        
        return tempResults.back();        
    }

    FFTransformer<T>* getFft() const
    {
        return m_fft;
    }

    void setFft(FFTransformer<T>* fft)
    {
        this->m_fft = fft;
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

    std::vector<Complex<T>**> getKernelStates() const
    {
        return m_kernelStates;
    }

    void setKernelStates(std::vector<Complex<T>**> kernelStates)
    {
        this->m_kernelStates = kernelStates;
    }

    std::vector<Kernel<T>*> getKernels() const
    {
        return m_kernels;
    }

    void setKernels(std::vector<Kernel<T>*> kernels)
    {
        this->m_kernels = kernels;
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


#endif	/* FFTIMAGEFILTERBATCHITEM_H */

