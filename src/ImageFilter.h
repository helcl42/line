/* 
 * File:   ImageFilter.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:03 AM
 */

#ifndef IMAGEFILTER_H
#define	IMAGEFILTER_H

#include "Image.h"

template <class T>
class ImageFilter
{
private:
    Image<T>* m_image;

public:
    ImageFilter(Image<T>* image);

    ~ImageFilter() {}

    void smooth();

    void gaussianBlur();

    void sharpen();
};


template <class T>
void ImageFilter<T>::smooth()
{
    const double m = 1.0 / 9;
    double result;

    Pixel<T>* pixel = NULL;

    for (unsigned int y = 1; y < m_image->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_image->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_image->getPixel(y, x);

                result = m * m_image->getPixelChannelValue(y - 1, x - 1, ch) +
                        m * m_image->getPixelChannelValue(y - 1, x, ch) +
                        m * m_image->getPixelChannelValue(y - 1, x + 1, ch) +
                        m * m_image->getPixelChannelValue(y, x - 1, ch) +
                        m * m_image->getPixelChannelValue(y, x, ch) +
                        m * m_image->getPixelChannelValue(y, x + 1, ch) +
                        m * m_image->getPixelChannelValue(y + 1, x - 1, ch) +
                        m * m_image->getPixelChannelValue(y + 1, x, ch) +
                        m * m_image->getPixelChannelValue(y + 1, x + 1, ch);

                switch (ch)
                {
                    case 0:
                        pixel->r = result;
                        break;
                    case 1:
                        pixel->g = result;
                        break;
                    case 2:
                        pixel->b = result;
                        break;
                }
            }
            m_image->setPixelValue(y, x, pixel);
        }
    }
}

template<class T>
void ImageFilter<T>::gaussianBlur()
{
    const double m = 1.0 / 16;
    double result;

    Pixel<float>* pixel = NULL;

    for (unsigned int y = 1; y < m_image->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_image->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_image->getPixel(y, x);

                result = m * m_image->getPixelChannelValue(y - 1, x - 1, ch) +
                        m * 2.0 * m_image->getPixelChannelValue(y - 1, x, ch) +
                        m * m_image->getPixelChannelValue(y - 1, x + 1, ch) +
                        m * 2.0 * m_image->getPixelChannelValue(y, x - 1, ch) +
                        m * 4.0 * m_image->getPixelChannelValue(y, x, ch) +
                        m * 2.0 * m_image->getPixelChannelValue(y, x + 1, ch) +
                        m * m_image->getPixelChannelValue(y + 1, x - 1, ch) +
                        m * 2.0 * m_image->getPixelChannelValue(y + 1, x, ch) +
                        m * m_image->getPixelChannelValue(y + 1, x + 1, ch);

                switch (ch)
                {
                    case 0:
                        pixel->r = result;
                        break;
                    case 1:
                        pixel->g = result;
                        break;
                    case 2:
                        pixel->b = result;
                        break;
                }
            }
            m_image->setPixelValue(y, x, pixel);
        }
    }
}

/**
 * 
 */
template <class T>
void ImageFilter<T>::sharpen()
{
    const double m = 1.0 / 3;
    double result;

    Pixel<T>* pixel = NULL;

    for (unsigned int y = 1; y < m_image->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_image->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_image->getPixel(y, x);

                result =
                        m * -2.0 * m_image->getPixelChannelValue(y - 1, x, ch) +
                        m * -2.0 * m_image->getPixelChannelValue(y, x - 1, ch) +
                        m * 11.0 * m_image->getPixelChannelValue(y, x, ch) +
                        m * -2.0 * m_image->getPixelChannelValue(y, x + 1, ch) +
                        m * -2.0 * m_image->getPixelChannelValue(y + 1, x, ch);

                switch (ch)
                {
                    case 0:
                        pixel->r = result;
                        break;
                    case 1:
                        pixel->g = result;
                        break;
                    case 2:
                        pixel->b = result;
                        break;
                }
            }
            m_image->setPixelValue(y, x, pixel);
        }
    }
}

#endif	/* IMAGEFILTER_H */

