#include "PrewittStrategy.h"
#include "DetectionParams.h"

BestLine* PrewittStrategy::detectLine()
{        
    //replaintSimilarColorPlaces();    
    prewittAlgorithm();    
    gaussianBlur();
    traverseImage();    
    return findBestLine();
}

/**
 *    | 1   1   0 | 
 *    | 1   0  -1 |
 *    | 0  -1  -1 | 
 */
void PrewittStrategy::prewittAlgorithm()
{
    double min = 1.0;
    double max = 0.0;
    const double k = 1 /3;

    unsigned int imageHeight = m_workImage->getHeight();
    unsigned int imageWidth = m_workImage->getWidth();

    double* buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i += 1)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j += 1)
        {
            double gx = k *
                    Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1), m_workImage->getPixel(i + 1, j - 1)) +
                    Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j    ), m_workImage->getPixel(i + 1, j    )) +
                    Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j + 1), m_workImage->getPixel(i + 1, j + 1));

            double gy = k *
                    Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1), m_workImage->getPixel(i - 1, j + 1)) +
                    Pixel<float>::colourDifference(m_workImage->getPixel(i    , j - 1), m_workImage->getPixel(i    , j + 1)) +
                    Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j - 1), m_workImage->getPixel(i + 1, j + 1));

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

            m_workImage->setPixelValue(y, x, &pixel);
        }
    }
    SAFE_DELETE_ARRAY(buffer);
}
