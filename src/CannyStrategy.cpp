#include "CannyStrategy.h"
#include "DetectionParams.h"

BestLine* CannyStrategy::detectLine()
{        
    //replaintSimilarColorPlaces();    
    cannyAlgorithm();    
    gaussianBlur();
    traverseImage();    
    return findBestLine();
}

/**
 *          | 2   4   5   4   2 |
 *          | 4   9  12   9   4 | 
 *  1/159 * | 5  12  15  12   5 |
 *          | 4   9  12   9   4 |
 *          | 2   4   5   4   2 | 
 */
void CannyStrategy::cannyAlgorithm()
{
    double min = 1.0;
    double max = 0.0;
    const double k = 1 / 159;

    unsigned int imageHeight = m_workImage->getHeight();
    unsigned int imageWidth = m_workImage->getWidth();

    double* buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 2; i < imageHeight - 2; i++)
    {
        for (unsigned int j = 2; j < imageWidth - 2; j++)
        {
            double gx = k *
                    2 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j - 2), m_workImage->getPixel(i + 2, j - 2)) +
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 2), m_workImage->getPixel(i + 1, j - 2)) +
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i,     j - 2), m_workImage->getPixel(i,     j - 2)) +
            
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j - 1), m_workImage->getPixel(i + 2, j - 1)) +
                    9 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1), m_workImage->getPixel(i + 1, j - 1)) +
                   12 * Pixel<float>::colourDifference(m_workImage->getPixel(i,     j - 1), m_workImage->getPixel(i,     j - 1)) +
            
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j), m_workImage->getPixel(i + 2, j)) +
                   12 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j), m_workImage->getPixel(i + 1, j)) +
                   15 * Pixel<float>::colourDifference(m_workImage->getPixel(i,     j), m_workImage->getPixel(i,     j)) +
            
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j + 1), m_workImage->getPixel(i + 2, j + 1)) +
                    9 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j + 1), m_workImage->getPixel(i + 1, j + 1)) +
                   12 * Pixel<float>::colourDifference(m_workImage->getPixel(i,     j + 1), m_workImage->getPixel(i,     j + 1)) +
            
                    2 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j + 2), m_workImage->getPixel(i + 2, j + 2)) +
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j + 2), m_workImage->getPixel(i + 1, j + 2)) +
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i,     j + 2), m_workImage->getPixel(i,     j + 2));

            double gy = k *
                    2 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j - 2), m_workImage->getPixel(i - 2, j + 2)) +
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j - 1), m_workImage->getPixel(i - 2, j + 1)) +
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 2, j    ), m_workImage->getPixel(i - 2, j    )) +
            
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 2), m_workImage->getPixel(i - 1, j + 2)) +
                    9 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1), m_workImage->getPixel(i - 1, j + 1)) +
                   12 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j    ), m_workImage->getPixel(i - 1, j    )) +
            
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i, j - 2), m_workImage->getPixel(i, j + 2)) +
                   12 * Pixel<float>::colourDifference(m_workImage->getPixel(i, j - 1), m_workImage->getPixel(i, j + 1)) +
                   15 * Pixel<float>::colourDifference(m_workImage->getPixel(i, j    ), m_workImage->getPixel(i, j    )) +
            
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j - 2), m_workImage->getPixel(i + 1, j + 2)) +
                    9 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j - 1), m_workImage->getPixel(i + 1, j + 1)) +
                   12 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j    ), m_workImage->getPixel(i + 1, j    )) +
            
                    2 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 2, j - 2), m_workImage->getPixel(i + 2, j + 2)) +
                    4 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 2, j - 1), m_workImage->getPixel(i + 2, j + 1)) +
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 2, j    ), m_workImage->getPixel(i + 2, j    ));
                    

            double val = pow(gx * gx + gy * gy, 0.5);

            if (val > max) max = val;
            if (val < min) min = val;

            buffer[i * imageWidth + j] = val;
        }
    }

    PixelRGB<float> pixel;

    for (unsigned int y = 1; y < imageHeight - 1; y++)
    {
        for (unsigned int x = 1; x < imageWidth - 1; x++)
        {
            double val = (buffer[y * imageWidth + x] - min) / (max - min) * 255;            

            if (val > DetectionParams::colorTreshold)
            {
                pixel.r = 255;
                pixel.g = 255;
                pixel.b = 255;
            }
            else
            {
                pixel.r = 0;
                pixel.g = 0;
                pixel.b = 0;
            }
            
//            pixel.r = val;
//            pixel.g = val;
//            pixel.b = val;
          
            m_workImage->setPixelValue(y, x, &pixel);
        }
    }
    SAFE_DELETE_ARRAY(buffer);
}

