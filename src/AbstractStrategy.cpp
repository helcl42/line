#include <stdexcept>

#include "AbstractStrategy.h"

void AbstractStrategy::blur()
{
    const double m = 1.0 / 9;
    double result;

    Pixel<float>* pixel = new PixelRGB<float>();

    for (unsigned int y = 1; y < m_bmpImage->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_bmpImage->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                result = m * m_bmpImage->getPixelChannelValue(y - 1, x - 1, ch) +
                         m * m_bmpImage->getPixelChannelValue(y - 1, x    , ch) +
                         m * m_bmpImage->getPixelChannelValue(y - 1, x + 1, ch) +
                         m * m_bmpImage->getPixelChannelValue(y    , x - 1, ch) +
                         m * m_bmpImage->getPixelChannelValue(y    , x    , ch) +
                         m * m_bmpImage->getPixelChannelValue(y    , x + 1, ch) +
                         m * m_bmpImage->getPixelChannelValue(y + 1, x - 1, ch) +
                         m * m_bmpImage->getPixelChannelValue(y + 1, x    , ch) +
                         m * m_bmpImage->getPixelChannelValue(y + 1, x + 1, ch);

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
            m_bmpImage->setPixelValue(y, x, pixel);
        }
    }
    SAFE_DELETE(pixel);
}

void AbstractStrategy::replaintSimilarColorPlaces(int interval)
{
    if(m_settings == NULL) 
    {
        throw std::runtime_error("Settings NULL");
    }
        
    PixelRGB<float> pixelMinus;
    PixelRGB<float> pixelPlus;
    Pixel<float>* pixel = NULL;    

    pixelMinus.r = 
            m_settings->searchedColor.b > interval ? m_settings->searchedColor.b - interval : 0;
    pixelMinus.g = 
            m_settings->searchedColor.g > interval ? m_settings->searchedColor.g - interval : 0;
    pixelMinus.b = 
            m_settings->searchedColor.r > interval ? m_settings->searchedColor.r - interval : 0;

    pixelPlus.r = 
            m_settings->searchedColor.b + interval < 255 ? m_settings->searchedColor.b + interval : 255;
    pixelPlus.g = 
            m_settings->searchedColor.g + interval < 255 ? m_settings->searchedColor.g + interval : 255;
    pixelPlus.b =
            m_settings->searchedColor.r + interval < 255 ? m_settings->searchedColor.r + interval : 255;

    if (m_settings->searchedColor.r > m_settings->searchedColor.g)
    {
        if (m_settings->searchedColor.b > m_settings->searchedColor.r)
        {
            m_baseColor.r = 255;
            m_baseColor.g = 0;
            m_baseColor.b = 0;
        }
        else
        {
            m_baseColor.r = 0;
            m_baseColor.g = 0;
            m_baseColor.b = 255;
        }
    } 
    else
    {
        if (m_settings->searchedColor.g > m_settings->searchedColor.b)
        {
            m_baseColor.r = 0;
            m_baseColor.g = 255;
            m_baseColor.b = 0;
        } 
        else
        {
            m_baseColor.r = 255;
            m_baseColor.g = 0;
            m_baseColor.b = 0;
        }
    }

    for (unsigned int i = 0; i < m_bmpImage->getHeight(); ++i)
    {
        for (unsigned int j = 0; j < m_bmpImage->getWidth(); ++j)
        {
            pixel = m_bmpImage->getPixel(i, j);

            if (pixel->r <= pixelPlus.r && pixel->r >= pixelMinus.r
                    && pixel->g <= pixelPlus.g && pixel->g >= pixelMinus.g
                    && pixel->b <= pixelPlus.b && pixel->b >= pixelMinus.b)
            {
                pixel->r = m_baseColor.r;
                pixel->g = m_baseColor.g;
                pixel->b = m_baseColor.b;
            }
            else
            {
                pixel->r = 0;
                pixel->g = 0;
                pixel->b = 0;
            }
        }
    }    
}
