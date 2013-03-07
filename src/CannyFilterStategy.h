/* 
 * File:   CannyFilterStategy.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:39 PM
 */

#ifndef CANNYFILTERSTATEGY_H
#define	CANNYFILTERSTATEGY_H

#include "EdgeFilterStrategy.h"

template <class T>
class CannyFilterStrategy : public EdgeFilterStrategy<T>
{
public:
    CannyFilterStrategy()
    : EdgeFilterStrategy<T>() {}

    CannyFilterStrategy(Image<T>* image)
    : EdgeFilterStrategy<T>(image) {}

    virtual ~CannyFilterStrategy() {}

    void applyFilter(unsigned int threshold);

private:
    void cannyAlgorithm();
};

/**
 *          | 2   4   5   4   2 |
 *          | 4   9  12   9   4 | 
 *  1/159 * | 5  12  15  12   5 |
 *          | 4   9  12   9   4 |
 *          | 2   4   5   4   2 | 
 */
template <class T>
void CannyFilterStrategy<T>::cannyAlgorithm()
{
    this->m_min = 1.0;
    this->m_max = 0.0;
    const double k = 1 / 159;

    unsigned int imageHeight = this->m_image->getHeight();
    unsigned int imageWidth = this->m_image->getWidth();

    this->m_buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 2; i < imageHeight - 2; i++)
    {
        for (unsigned int j = 2; j < imageWidth - 2; j++)
        {
            double gx = k *
                    2 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j - 2), this->m_image->getPixel(i + 2, j - 2)) +
                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 2), this->m_image->getPixel(i + 1, j - 2)) +
                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j - 2), this->m_image->getPixel(i, j - 2)) +

                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j - 1), this->m_image->getPixel(i + 2, j - 1)) +
                    9 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1), this->m_image->getPixel(i + 1, j - 1)) +
                    12 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j - 1), this->m_image->getPixel(i, j - 1)) +

                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j), this->m_image->getPixel(i + 2, j)) +
                    12 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j), this->m_image->getPixel(i + 1, j)) +
                    15 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j), this->m_image->getPixel(i, j)) +

                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j + 1), this->m_image->getPixel(i + 2, j + 1)) +
                    9 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j + 1), this->m_image->getPixel(i + 1, j + 1)) +
                    12 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j + 1), this->m_image->getPixel(i, j + 1)) +

                    2 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j + 2), this->m_image->getPixel(i + 2, j + 2)) +
                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j + 2), this->m_image->getPixel(i + 1, j + 2)) +
                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j + 2), this->m_image->getPixel(i, j + 2));

            double gy = k *
                    2 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j - 2), this->m_image->getPixel(i - 2, j + 2)) +
                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j - 1), this->m_image->getPixel(i - 2, j + 1)) +
                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 2, j), this->m_image->getPixel(i - 2, j)) +

                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 2), this->m_image->getPixel(i - 1, j + 2)) +
                    9 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j - 1), this->m_image->getPixel(i - 1, j + 1)) +
                    12 * Pixel<float>::colourDifference(this->m_image->getPixel(i - 1, j), this->m_image->getPixel(i - 1, j)) +

                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j - 2), this->m_image->getPixel(i, j + 2)) +
                    12 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j - 1), this->m_image->getPixel(i, j + 1)) +
                    15 * Pixel<float>::colourDifference(this->m_image->getPixel(i, j), this->m_image->getPixel(i, j)) +

                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j - 2), this->m_image->getPixel(i + 1, j + 2)) +
                    9 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j - 1), this->m_image->getPixel(i + 1, j + 1)) +
                    12 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 1, j), this->m_image->getPixel(i + 1, j)) +

                    2 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 2, j - 2), this->m_image->getPixel(i + 2, j + 2)) +
                    4 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 2, j - 1), this->m_image->getPixel(i + 2, j + 1)) +
                    5 * Pixel<float>::colourDifference(this->m_image->getPixel(i + 2, j), this->m_image->getPixel(i + 2, j));


            double val = pow(gx * gx + gy * gy, 0.5);

            if (val > this->m_max) this->m_max = val;
            if (val < this->m_min) this->m_min = val;

            this->m_buffer[i * imageWidth + j] = val;
        }
    }
}


template <class T>
void CannyFilterStrategy<T>::applyFilter(unsigned int threshold)
{
    cannyAlgorithm();
    this->resolveThreshold(threshold);
    SAFE_DELETE_ARRAY(this->m_buffer);
}


#endif	/* CANNYFILTERSTATEGY_H */

