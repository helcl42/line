/* 
 * File:   FFTImageFilterBatch.h
 * Author: lubos
 *
 * Created on April 3, 2013, 3:29 PM
 */

#ifndef FFTIMAGEFILTERBATCH_H
#define	FFTIMAGEFILTERBATCH_H

#include "AbstractFFTImageFilter.h"
#include "FFTImageFilterBatchItem.h"

template <class T>
class FFTImageFilterBatch : public AbstractFFTImageFilter<T>
{
private:
    std::vector<FFTImageFilterBatchItem<float>*> m_filters;

public:

    FFTImageFilterBatch()
    : AbstractFFTImageFilter<T>()
    {        
    }

    FFTImageFilterBatch(ImageMap<T>* imageMap, std::vector<FFTImageFilterBatchItem<float>*>& filters)
    : AbstractFFTImageFilter<T>(imageMap)
    {
        this->m_filters = filters;
    }

    virtual ~FFTImageFilterBatch()
    {
        std::vector<FFTImageFilterBatchItem<float>*>::iterator ii;
        for (ii = m_filters.begin(); ii != m_filters.end(); ++ii)
        {
            SAFE_DELETE(*ii);
        }
        cleanUp();
    }

    void cleanUp()
    {
//        typename std::vector<FFTImageFilterBatchItem<T> *>::iterator ii;
//        for (ii = m_filters.begin(); ii != m_filters.end(); ++ii)
//        {
//            (*ii)->cleanUp();
//        }

//        for (unsigned int i = 0; i < this->m_height; i++)
//        {
//            SAFE_DELETE_ARRAY(this->m_fftState[i])
//        }
//        SAFE_DELETE_ARRAY(this->m_fftState);
    }

    void addItem(FFTImageFilterBatchItem<T>* filter)
    {
        m_filters.push_back(filter);
    }

    void setInstance(ImageMap<T>* image)
    {
        unsigned int size;
        
        if (this->m_fftState != NULL)
        {
            for (unsigned int i = 0; i < this->m_height; i++)
            {
                SAFE_DELETE_ARRAY(this->m_fftState[i])
            }
            SAFE_DELETE_ARRAY(this->m_fftState);
        }

        if (this->m_height != image->getHeight() || this->m_width != image->getWidth())
        {                        
            this->m_originalHeight = image->getHeight();
            this->m_originalWidth = image->getWidth();

            size = this->m_originalWidth > this->m_originalHeight ? this->m_originalWidth : this->m_originalHeight;
            this->m_width = this->m_height = FFTransformer<T>::computeImageSize(size);

            this->m_fft->setOriginalHeight(this->m_originalHeight);
            this->m_fft->setOriginalWidth(this->m_originalWidth);
            this->m_fft->setHeight(this->m_height);
            this->m_fft->setWidth(this->m_width);           
        }

        this->m_imageMap = image;       
        
        this->m_fftState = this->m_fft->forwardFFT(image);

        typename std::vector<FFTImageFilterBatchItem<T> *>::iterator ii;
        for (ii = m_filters.begin(); ii != m_filters.end(); ++ii)
        {            
            (*ii)->setInstance(image, this->m_fft);
        }
    }

    void applyFilter()
    {
        Complex<T>** tempResult = this->m_fftState;
        Complex<T>** res = NULL;
        
        if(m_filters.size() == 0) return;
        
        for (unsigned int i = 0; i < m_filters.size(); i++)
        {            
            res = m_filters[i]->applyFilterItem(tempResult);

            for (unsigned int j = 0; j < this->m_height; j++)
            {
                SAFE_DELETE_ARRAY(tempResult[j]);
            }

            tempResult = res;
        }

        if (this->m_filters.size() % 2 == 0)
        {
            this->m_fft->inverseFFT(res, this->m_imageMap, false);
        }
        else
        {
            this->m_fft->inverseFFT(res, this->m_imageMap, true);
        }

        if (res != NULL)
        {
            for (unsigned int j = 0; j < this->m_height; j++)
            {
                SAFE_DELETE_ARRAY(res[j]);
            }
        }
    }
};

#endif	/* FFTIMAGEFILTERBATCH_H */

