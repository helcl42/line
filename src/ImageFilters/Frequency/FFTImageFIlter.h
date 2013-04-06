/* 
 * File:   FFTImageFilter.h
 * Author: lubos
 *
 * Created on April 3, 2013, 3:18 AM
 */

#ifndef FFTIMAGEFILTER_H
#define	FFTIMAGEFILTER_H

#include "../Complex.h"
#include "../Kernel/Kernel.h"
#include "../FFTransformer.h"
#include "AbstractFFTImageFilter.h"

template <class T>
class FFTImageFilter : public AbstractFFTImageFilter<T>
{
private:    
    std::vector<Complex<T>**> m_kernelStates;

    std::vector<Kernel<T>* > m_kernels;

public:

    FFTImageFilter(Kernel<T>* kernelX)
    : AbstractFFTImageFilter<T>()
    {
        m_kernels.push_back(kernelX);        
    }

    FFTImageFilter(std::vector<Kernel<T>*>& kernels)
    : AbstractFFTImageFilter<T>()
    {
        typename std::vector<Kernel<T>* >::iterator ii;

        for (ii = kernels.begin(); ii != kernels.end(); ++ii)
        {
            m_kernels.push_back(*ii);
        }        
    }

    FFTImageFilter(ImageMap<T>* image, Kernel<T>* kernelX)
    : AbstractFFTImageFilter<T>(image)
    {                
        m_kernels.push_back(kernelX);

        allocateStructures(image);
        this->m_fftState = this->m_fft->forwardFFT(image);       
    }

    FFTImageFilter(ImageMap<T>* image, std::vector<Kernel<T>* >& kernels)
    : AbstractFFTImageFilter<T>(image)
    {        
        typename std::vector<Kernel<T> *>::iterator ii;
        for (ii = kernels.begin(); ii != kernels.end(); ++ii)
        {
            m_kernels.push_back(*ii);
        }        

        allocateStructures(image);

        this->m_fftState = this->m_fft->forwardFFT(image);       
    }

    virtual ~FFTImageFilter()
    {
        typename std::vector<Kernel<T> *>::iterator ii;
        for (ii = m_kernels.begin(); ii != m_kernels.end(); ++ii)
        {
            SAFE_DELETE(*ii);
        }
        
        typename std::vector<Complex<T>**>::iterator jj;
        for (jj = m_kernelStates.begin(); jj != m_kernelStates.end(); ++jj)
        {
            for (unsigned int i = 0; i < this->m_height; i++)
            {
                SAFE_DELETE_ARRAY((*jj)[i]);
            }
            SAFE_DELETE_ARRAY(*jj);
        }
    }

    void cleanUp()
    {
        typename std::vector<Complex<T>**>::iterator jj;
        for (jj = m_kernelStates.begin(); jj != m_kernelStates.end(); ++jj)
        {
            for (unsigned int i = 0; i < this->m_height; i++)
            {
                SAFE_DELETE_ARRAY((*jj)[i]);
            }
            SAFE_DELETE_ARRAY(*jj);
        }

        for (unsigned int i = 0; i < this->m_height; i++)
        {
            SAFE_DELETE_ARRAY(this->m_fftState[i])
        }
        SAFE_DELETE_ARRAY(this->m_fftState);
    }

    void allocateStructures(ImageMap<float>* image)
    {
        ImageMap<T>* imageMap = NULL;
        typename std::vector<Kernel<T> *>::iterator ii;
        for (ii = m_kernels.begin(); ii != m_kernels.end(); ++ii)
        {
            imageMap = new ImageMap<T > (image->getHeight(), image->getWidth(), *ii);
            m_kernelStates.push_back(this->m_fft->forwardFFT(imageMap));
            SAFE_DELETE(imageMap);
        }
    }

    void setInstance(ImageMap<T>* image)
    {
        unsigned int size;

        if (this->m_height != image->getHeight() || this->m_width != image->getWidth())
        {
            this->m_originalHeight = image->getHeight();
            this->m_originalWidth = image->getWidth();

            size = this->m_originalWidth > this->m_originalHeight ? this->m_originalWidth : this->m_originalHeight;
            this->m_width = this->m_height = FFTransformer<float>::computeImageSize(size);

            this->m_fft->setHeight(this->m_height);
            this->m_fft->setWidth(this->m_width);
            this->m_fft->setOriginalHeight(this->m_originalHeight);
            this->m_fft->setOriginalWidth(this->m_originalWidth);

            cleanUp();
            allocateStructures(image);
        }
        
        this->m_imageMap = image;

        this->m_fftState = this->m_fft->forwardFFT(image);
    }

    void applyFilter()
    {
        std::vector<Complex<T>**> tempResults;
        typename std::vector<Complex<T>**>::iterator ii;

        if (m_kernelStates.size() > 0)
        {
            for (ii = m_kernelStates.begin(); ii != m_kernelStates.end(); ++ii)
            {
                this->m_fft->convolve(this->m_fftState, *ii);
                tempResults.push_back(*ii);
            }

            if (tempResults.size() > 1)
            {
                for (unsigned int i = 0; i < tempResults.size() - 1; i++)
                {
                    this->m_fft->add(tempResults[i + 1], tempResults[i]);
                }
            }

            this->m_fft->inverseFFT(tempResults.back(), this->m_imageMap, true);

            for (ii = tempResults.begin(); ii != tempResults.end(); ++ii)
            {
                for (unsigned int j = 0; j < this->m_height; j++)
                {
                    SAFE_DELETE_ARRAY((*ii)[j])
                }
            }
        }
        else
        {
            this->m_fft->inverseFFT(this->m_fftState, this->m_imageMap, false);
        }
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
};

#endif	/* FFTIMAGEFILTER_H */

