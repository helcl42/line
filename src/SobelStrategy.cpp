#include "SobelStrategy.h"
#include "DetectionParams.h"

LinePair* SobelStrategy::detectLine()
{        
    //replaintSimilarColorPlaces();    
    sobelAlgorithm();    
    gaussianBlur();    
    traverseImage();    
    return findBestLine();
}

Rectangle* SobelStrategy::detectRectangle()
{
    return NULL;
}

void SobelStrategy::sobelAlgorithm()
{
    double min = 1.0;
    double max = 0.0;

    unsigned int imageHeight = m_workImage->getHeight();
    unsigned int imageWidth = m_workImage->getWidth();

    double* buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i += 1)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j += 1)
        {
            double gx =
                    Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1),
                    m_workImage->getPixel(i + 1, j - 1)) +
                    2.0 * Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j),
                    m_workImage->getPixel(i + 1, j)) +
                    Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j + 1),
                    m_workImage->getPixel(i + 1, j + 1));

            double gy =
                    Pixel<float>::colourDifference(m_workImage->getPixel(i - 1, j - 1),
                    m_workImage->getPixel(i - 1, j + 1)) +
                    2.0 * Pixel<float>::colourDifference(m_workImage->getPixel(i, j - 1),
                    m_workImage->getPixel(i, j + 1)) +
                    Pixel<float>::colourDifference(m_workImage->getPixel(i + 1, j - 1),
                    m_workImage->getPixel(i + 1, j + 1));

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
