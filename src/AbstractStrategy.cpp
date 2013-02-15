#include "AbstractStrategy.h"

AbstractStrategy::AbstractStrategy(BmpImage<float>* image, DetectionSettings* settings)
: m_bmpImage(image), m_settings(settings)
{
    setBaseColor();
}

AbstractStrategy::~AbstractStrategy()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();
}

/**
 * simple filter to join separated pixels
 * after edge detection to make lower constrast
 * between left pixels
 */
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

/**
 * simple filter to join separated pixels
 * after edge detection to make lower constrast
 * between left pixels
 */
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

/**
 * //TODO:)
 */
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

/**
 * private function sets base color from settings
 * structure should be called in constructor only
 */
void AbstractStrategy::setBaseColor()
{
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
}

/**
 * function replaces pixels with similar color by the nearest
 * base color R, G or B
 * @param interval
 */
void AbstractStrategy::replaintSimilarColorPlaces(int interval)
{
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

/**
 * function chooses from four founded
 * lines from four traverse image calls
 * @param lines
 * @return 
 */
bool AbstractStrategy::storeBestLine(Line** lines)
{
    int maxIndex = Line::getMaxLengthIndex(lines);
    Line* best = lines[maxIndex];
    //    Line* temp = NULL;

    for (int i = 0; i < 4; i++)
    {
        if (maxIndex != i)
        {
            SAFE_DELETE(lines[i]);
        }
    }

    if (best->points.size() > LINE_LENGTH_TRESHOLD)
    {
        //        for (unsigned int i = 0; i < m_lines.size(); ++i)
        //        {
        //            temp = m_lines[i];
        //            if (best->isSimilar(temp) && temp->points.size() < best->points.size())
        //            {
        //                SAFE_DELETE(temp);
        //                m_lines.erase(m_lines.begin() + i);
        //            }
        //        }

        m_lines.push_back(best);
        return true;
    }
    else
    {
        SAFE_DELETE(best);
        return false;
    }
}

/**
 * function traverses image in matrix way
 * and calls inner traverse function with proper
 * vector parameters and actual position in image
 * @return 
 */
Line* AbstractStrategy::traverseImage()
{
    Pixel<float>* pixel = NULL;
    Line * lines[4];

    for (unsigned int i = 1; i < m_bmpImage->getHeight() - 1; ++i)
    {
        for (unsigned int j = 1; j < m_bmpImage->getWidth() - 1; ++j)
        {
            pixel = m_bmpImage->getPixel(i, j);
            //if(pixel->r >= m_baseColor.r && pixel->g >= m_baseColor.g && pixel->b >= m_baseColor.b)            
            if (pixel->r > COLOR_TRESHOLD || pixel->b > COLOR_TRESHOLD || pixel->g > COLOR_TRESHOLD)
            {
                lines[0] = findCorrectLine(1, 0, 0, 1, i, j);
                lines[1] = findCorrectLine(-1, 0, 0, 1, i, j);

                lines[2] = findCorrectLine(0, 1, 1, 0, i, j);
                lines[3] = findCorrectLine(0, -1, 1, 0, i, j);

                if (storeBestLine(lines))
                {
                    j += m_bmpImage->getWidth() / 15;
                    //break;
                }
            }
        }
    }

    if (m_lines.size() == 0)
    {
        return NULL;
    }

    Line* ret = getLongestLine();
    writeLineInImage(ret, 255, 0, 0);
    removeSimilarLines(ret);
    Line* similar = findLineWithSimilarDirection(ret);
    writeLineInImage(similar, 0, 0, 255);

    return ret;
}

/**
 * function removes Lines containing similar points
 * to the first longest line
 * @param input
 */
void AbstractStrategy::removeSimilarLines(Line* input)
{
    Line* temp = NULL;

    if (input == NULL)
        return;

    for (unsigned int index = 0; index < m_lines.size(); ++index)
    {
        temp = m_lines[index];
        if (input->isSimilar(temp))
        {
            SAFE_DELETE(temp);
            m_lines.erase(m_lines.begin() + index);
            index--;
        }
    }
}

/**
 * writes complete line structure to imageMatrix in selected color
 * @param line
 * @param r
 * @param g
 * @param b
 */
void AbstractStrategy::writeLineInImage(Line* line, int r, int g, int b)
{
    Vector2<int> linePoint;

    if (line == NULL)
        return;

    for (unsigned int i = 0; i < line->points.size(); i++)
    {
        linePoint = line->points[i];
        m_bmpImage->setPixelValue(linePoint.y, linePoint.x, r, g, b);
    }
}

/**
 * finds neither line with similar direction or implicitly uses
 * the longest line, if non of lines passes test by multiply 
 * by magicConstant
 * @param input
 * @return Line*
 */
Line* AbstractStrategy::findLineWithSimilarDirection(Line* input)
{
    if (input == NULL)
        return NULL;

    Vector2<int> beginPoint = input->points[0];
    Vector2<int> endPoint = input->points[input->points.size() - 1];

    double minDelta = 10000.0;
    double delta = 0;
    double magicConstant = 1.5;
    Line* bestLine = getLongestLine();
    bool longestIsBest = true;
    double k = (double) (endPoint.y - beginPoint.y) / (double) (endPoint.x - beginPoint.x);

    std::vector<Line*>::const_iterator ii;
    for (ii = m_lines.begin(); ii != m_lines.end(); ++ii)
    {
        Vector2<int> tempBeginPoint = (*ii)->points[0];
        Vector2<int> tempEndPoint = (*ii)->points[(*ii)->points.size() - 1];

        double testedK = (double) (tempEndPoint.y - tempBeginPoint.y) / (double) (tempEndPoint.x - tempBeginPoint.x);

        //      std::cout << "testedK = " << testedK << " k = " << k << std::endl;        
        if (testedK > k)
        {
            delta = testedK - k;
        }
        else
        {
            delta = k - testedK;
        }

        if (delta * magicConstant < minDelta)
        {
            if (longestIsBest)
            {
                SAFE_DELETE(bestLine);
                longestIsBest = false;
            }
            minDelta = delta;
            bestLine = *ii;
        }
    }

    return bestLine;
}

/**
 * function represents inner traverse of image
 * for pixels that passes color treshold test
 * @param vecY
 * @param vecX
 * @param chY
 * @param chX
 * @param posY
 * @param posX
 * @return 
 */
Line* AbstractStrategy::findCorrectLine(int vecY, int vecX, int chY, int chX, unsigned int posY, unsigned int posX)
{
    Pixel<float>* pixel = NULL;
    Line* line = new Line();

    int countOfFails = 0;
    int vectorY = vecY;
    int vectorX = vecX;

    line->points.push_back(Vector2<int>(posX, posY));

    while (posY > 2 && posX > 2 && posY < m_bmpImage->getHeight() - 2 && posX < m_bmpImage->getWidth() - 2)
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
            line->points.push_back(Vector2<int>(posX, posY));
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

/**
 * returned pointer of the longest line
 * should be deleted manually
 * @return 
 */
Line* AbstractStrategy::getLongestLine()
{
    Line* longest = NULL;
    unsigned int maxLineSize = 0;
    unsigned int longestIndex = 0;

    if (m_lines.size() > 0)
    {
        for (unsigned int i = 0, tempSize = 0; i < m_lines.size(); i++)
        {
            tempSize = m_lines[i]->points.size();
            if (tempSize > maxLineSize)
            {
                longest = m_lines[i];
                longestIndex = i;
                maxLineSize = tempSize;
            }
        }
        m_lines.erase(m_lines.begin() + longestIndex);
    }
    return longest;
}
