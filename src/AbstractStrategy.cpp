#include "AbstractStrategy.h"

void AbstractStrategy::smooth()
{
    const double m = 1.0 / 9;
    double result;

    Pixel<float>* pixel = NULL;

    for (unsigned int y = 1; y < m_bmpImage->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_bmpImage->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_bmpImage->getPixel(y, x);

                result = m * m_bmpImage->getPixelChannelValue(y - 1, x - 1, ch) +
                        m * m_bmpImage->getPixelChannelValue(y - 1, x, ch) +
                        m * m_bmpImage->getPixelChannelValue(y - 1, x + 1, ch) +
                        m * m_bmpImage->getPixelChannelValue(y, x - 1, ch) +
                        m * m_bmpImage->getPixelChannelValue(y, x, ch) +
                        m * m_bmpImage->getPixelChannelValue(y, x + 1, ch) +
                        m * m_bmpImage->getPixelChannelValue(y + 1, x - 1, ch) +
                        m * m_bmpImage->getPixelChannelValue(y + 1, x, ch) +
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
}

void AbstractStrategy::gaussianBlur()
{
    const double m = 1.0 / 16;
    double result;

    Pixel<float>* pixel = NULL;

    for (unsigned int y = 1; y < m_bmpImage->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_bmpImage->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_bmpImage->getPixel(y, x);

                result = m * m_bmpImage->getPixelChannelValue(y - 1, x - 1, ch) +
                        m * 2.0 * m_bmpImage->getPixelChannelValue(y - 1, x, ch) +
                        m * m_bmpImage->getPixelChannelValue(y - 1, x + 1, ch) +
                        m * 2.0 * m_bmpImage->getPixelChannelValue(y, x - 1, ch) +
                        m * 4.0 * m_bmpImage->getPixelChannelValue(y, x, ch) +
                        m * 2.0 * m_bmpImage->getPixelChannelValue(y, x + 1, ch) +
                        m * m_bmpImage->getPixelChannelValue(y + 1, x - 1, ch) +
                        m * 2.0 * m_bmpImage->getPixelChannelValue(y + 1, x, ch) +
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
}

void AbstractStrategy::sharpen()
{
    const double m = 1.0 / 3;
    double result;

    Pixel<float>* pixel = NULL;

    for (unsigned int y = 1; y < m_bmpImage->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_bmpImage->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_bmpImage->getPixel(y, x);

                result =
                        m * -2.0 * m_bmpImage->getPixelChannelValue(y - 1, x, ch) +
                        m * -2.0 * m_bmpImage->getPixelChannelValue(y, x - 1, ch) +
                        m * 11.0 * m_bmpImage->getPixelChannelValue(y, x, ch) +
                        m * -2.0 * m_bmpImage->getPixelChannelValue(y, x + 1, ch) +
                        m * -2.0 * m_bmpImage->getPixelChannelValue(y + 1, x, ch);

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
}

void AbstractStrategy::replaintSimilarColorPlaces(int interval)
{
    if (m_settings == NULL)
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

Line* AbstractStrategy::traverseImage()
{
    Pixel<float>* pixel = NULL;
    Line* line = NULL;
    Line* line2 = NULL;
    std::vector<Line*> lines;

    for (unsigned int i = 1; i < m_bmpImage->getHeight() - 1; ++i)
    {
        for (unsigned int j = 1; j < m_bmpImage->getWidth() - 1; ++j)
        {
            pixel = m_bmpImage->getPixel(i, j);
            //if(pixel->r >= m_baseColor.r && pixel->g >= m_baseColor.g && pixel->b >= m_baseColor.b)            
            if (pixel->r > COLOR_TRESHOLD || pixel->b > COLOR_TRESHOLD || pixel->g > COLOR_TRESHOLD)
            {
                line = findCorrectLine(1, 0, i, j);
                line2 = findCorrectLine(-1, 0, i, j);

                unsigned int size1 = line->points.size();
                unsigned int size2 = line2->points.size();
                if (size1 > size2)
                {
                    if (size1 > LINE_LENGTH_TRESHOLD)
                    {                        
                        lines.push_back(line);
                        break;
                    }
                    else
                    {
                        SAFE_DELETE(line);
                    }
                    SAFE_DELETE(line2);
                }
                else
                {
                    if (size2 > LINE_LENGTH_TRESHOLD)
                    {                        
                        lines.push_back(line2);
                        break;
                    }
                    else
                    {
                        SAFE_DELETE(line2);
                    }
                    SAFE_DELETE(line);
                }
            }
        }
    }

    if (lines.size() == 0)
    {
        return NULL;
    }

    Line* ret = getLongestLine(lines);

    for (unsigned int i = 0; i < lines.size(); ++i)
    {
        SAFE_DELETE(lines[i]);
    }
    lines.clear();
    
    return ret;
}

Line* AbstractStrategy::findCorrectLine(int vecY, int vecX, unsigned int posY, unsigned int posX)
{
    Pixel<float>* pixel = NULL;
    Line* line = new Line();

    int chY = 0;
    int chX = 1;

    int countOfFails = 0;
    int vectorY = vecY;
    int vectorX = vecX;

    Vector2<int> point;
    point.y = posY;
    point.x = posX;
    line->points.push_back(point);

    while (posY > 1 && posX > 1 && posY < m_bmpImage->getHeight() - 2 && posX < m_bmpImage->getWidth() - 2)
    {
        posY += vectorY;
        posX += vectorX;

        pixel = m_bmpImage->getPixel(posY, posX);

        //if(pixel->r >= m_baseColor.r && pixel->g >= m_baseColor.g && pixel->b >= m_baseColor.b)
        if (pixel->r > COLOR_TRESHOLD || pixel->b > COLOR_TRESHOLD || pixel->g > COLOR_TRESHOLD)
        {
            if (countOfFails > 0)
            {
                vectorY = vecY;
                vectorX = vecX;
            }
            countOfFails = 0;
            point.y = posY;
            point.x = posX;
            line->points.push_back(point);
        }
        else
        {
            if (countOfFails > 0)
                return line;

            //change direction and recover positons
            posY -= vectorY;
            posX -= vectorX;
            countOfFails++;
            vectorY = chY;
            vectorX = chX;
        }
    }
    return line;
}

Line* AbstractStrategy::getLongestLine(std::vector<Line*>& lines)
{
    Line* temp = NULL;
    Line* ret = new Line();
    unsigned int maxLineSize = 0;

    for (unsigned int i = 0, tempSize = 0; i < lines.size(); i++)
    {
        tempSize = lines[i]->points.size();
        if (tempSize > maxLineSize)
        {
            temp = lines[i];
            maxLineSize = tempSize;
        }
    }

    Vector2<int> linePoint;
    for (unsigned int i = 0; i < temp->points.size(); i++)
    {
        linePoint = temp->points[i];
        m_bmpImage->setPixelValue(linePoint.y, linePoint.x, 255, 0, 0);
        ret->points.push_back(linePoint);
    }

    return ret;
}