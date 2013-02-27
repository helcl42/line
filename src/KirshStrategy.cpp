#include "KirshStrategy.h"
#include "DetectionParams.h"


LinePair* KirshStrategy::detectLine()
{    
    //replaintSimilarColorPlaces();    
    kirshAlgorithm(); 
    //gaussianBlur();
    traverseImage();    
    return findBestLine();
}


/** 
 *  |  5   5  -3 | 
 *  |  5   0  -3 |
 *  | -3  -3  -3 |    
 */
void KirshStrategy::kirshAlgorithm()
{
    double min = 1.0;
    double max = 0.0;
    const double k = 1/15;

    unsigned int imageHeight = m_workImage->getHeight();
    unsigned int imageWidth = m_workImage->getWidth();

    double* buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i++)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j++)
        {
            double gx = k *                    
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1), m_workImage->getPixel(i, j)) 
                  - 5 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j    ), m_workImage->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j + 1), m_workImage->getPixel(i, j))
                  + 5 * Pixel<float>::colourDifference(m_workImage->getPixel(i    , j - 1), m_workImage->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i    , j + 1), m_workImage->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j - 1), m_workImage->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j    ), m_workImage->getPixel(i, j)) 
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j + 1), m_workImage->getPixel(i, j));            

            double gy = k *
                    5 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1), m_workImage->getPixel(i, j)) 
                  + 5 * Pixel<float>::colourDifference(m_workImage->getPixel(i    , j - 1), m_workImage->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j - 1), m_workImage->getPixel(i, j)) 
                  - 5 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j    ), m_workImage->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j    ), m_workImage->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j + 1), m_workImage->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i    , j + 1), m_workImage->getPixel(i, j))
                  - 3 * Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j + 1), m_workImage->getPixel(i, j));

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

