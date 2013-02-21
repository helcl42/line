#include "AbstractStrategy.h"
#include "Timer.h"

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

void AbstractStrategy::setBaseColor()
{
    float maxValue = 0;
    int maxIndex = 0;
    float temp;

    for (int i = 0; i < 3; i++)
    {
        temp = m_settings->searchedColor[i];
        if (temp > maxValue)
        {
            maxValue = temp;
            maxIndex = i;
        }
    }

    for (int i = 0; i < 3; i++)
    {
        if (i == maxIndex)
        {
            m_baseColor[i] = 255;
        }
        else
        {
            m_baseColor[i] = 0;
        }
    }
}

void AbstractStrategy::replaintSimilarColorPlaces(int interval)
{
    PixelRGB<float> pixelMinus;
    PixelRGB<float> pixelPlus;
    Pixel<float>* pixel = NULL;

    for (int i = 0; i < 3; i++)
    {
        pixelMinus[i] = m_settings->searchedColor[i] > interval ? m_settings->searchedColor[i] - interval : 0;
        pixelPlus[i] = m_settings->searchedColor[i] + interval < 255 ? m_settings->searchedColor[i] + interval : 255;
    }

    for (unsigned int i = 0; i < m_bmpImage->getHeight(); ++i)
    {
        for (unsigned int j = 0; j < m_bmpImage->getWidth(); ++j)
        {
            pixel = m_bmpImage->getPixel(i, j);

            if (pixel->isInInterval(&pixelMinus, &pixelPlus))
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

bool AbstractStrategy::storeBestLine(Line** lines)
{
    for (int i = 0; i < 4; i++)
    {
        if (lines[i]->computeLength() < LINE_LENGTH_TRESHOLD)
        {
            SAFE_DELETE(lines[i]);
        }
    }

    //int maxIndex = Line::getMaxLengthIndex(lines);
    Line* best = Line::getStraightesstLine(lines);
    if (best != NULL)
    {        
        for (int i = 0; i < 4; i++)
        {
            if (best != lines[i])
            {
                SAFE_DELETE(lines[i]);
            }
        }

        best->computeProperties();
        if (best->straightnessFactor < STRAIGHTNESS_TRESHOLD)
        {
            //std::cout << "store?? " << best->points.size() << " index " << maxIndex << " factor " << best->getStraightnessFactor() << std::endl;
            writeLineInImage(best, 255, 255, 0);
            m_lines.push_back(best);
            return true;
        }
        else
        {
            std::cout << "FAIL factor = " << best->straightnessFactor << " len = " << best->length << std::endl;
            SAFE_DELETE(best);
            return false;
        }
    }
    else
    {
        return false;
    }
}

void AbstractStrategy::traverseImage()
{
    Pixel<float>* pixel = NULL;
    Line * lines[4];

    for (unsigned int i = 1; i < m_bmpImage->getHeight() - 1; ++i)
    {
        for (unsigned int j = 1; j < m_bmpImage->getWidth() - 1; ++j)
        {
            pixel = m_bmpImage->getPixel(i, j);
            //if(pixel->r >= m_baseColor.r && pixel->g >= m_baseColor.g && pixel->b >= m_baseColor.b)            
            if (pixel->r > SELECTION_TRESHOLD || pixel->b > SELECTION_TRESHOLD || pixel->g > SELECTION_TRESHOLD)
            {
                lines[0] = findCorrectLine(1, 0, 0, 1, i, j);
                lines[1] = findCorrectLine(-1, 0, 0, 1, i, j);

                lines[2] = findCorrectLine(0, 1, 1, 0, i, j);
                lines[3] = findCorrectLine(0, -1, 1, 0, i, j);

                if (storeBestLine(lines))
                {
                    j += m_bmpImage->getWidth() / 15;
                    if (j < 1) j++;
                    //break;
                }
            }
        }
    }
}

void AbstractStrategy::setBestLine(Line* line1, Line* line2)
{
    m_bestLine[0] = line1;
    m_bestLine[1] = line2;
}

//todo rewrite
//void AbstractStrategy::sortLinesByStraightness()
//{
//    Line* temp = NULL;
//    Timer t;
//    t.start();
//    for(unsigned int i = 0; i < m_lines.size(); i++)
//    {
//        for(unsigned int j = 0; j < m_lines.size() - 1; j++)
//        {
//            if(m_lines[j]->length < m_lines[j + 1]->length)
//            {
//                temp = m_lines[j];
//                m_lines[j] = m_lines[j + 1];
//                m_lines[j + 1] = temp;
//            }
//        }
//    }
//    t.stop();
//    t.logTime();
//    
//     for (unsigned int i = 1; i < m_lines.size(); i++)
//    {
//        std::cout << "Line: " << m_lines[i]->points.front() << " " << m_lines[i]->points.back() << " len = " << m_lines[i]->length << std::endl;
//    }
//}

void AbstractStrategy::sortLinesByLength()
{    
    Line* temp = NULL;
    int j;

    for (unsigned int i = 1; i < m_lines.size(); i++)
    {
        temp = m_lines[i];
        for (j = i - 1; j >= 0; j--)
        {
            if (m_lines[j]->length > temp->length)
            {
                break;
            }
            m_lines[j + 1] = m_lines[j];
        }
        m_lines[j + 1] = temp;
    } 
}

void AbstractStrategy::sortLinesByStraightness()
{
    //Timer t;
    //t.start();
    Line* temp = NULL;
    int j;

    for (unsigned int i = 1; i < m_lines.size(); i++)
    {
        temp = m_lines[i];
        for (j = i - 1; j >= 0; j--)
        {
            if (m_lines[j]->straightnessFactor < temp->straightnessFactor)
            {
                break;
            }
            m_lines[j + 1] = m_lines[j];
        }
        m_lines[j + 1] = temp;
    }
    //t.stop();
    //t.logTime();           
        
//    for (unsigned int i = 1; i < m_lines.size(); i++)
//    {
//        std::cout << "Line: " << m_lines[i]->points.front() << " " << m_lines[i]->points.back() << " len = " << m_lines[i]->length << " straightness =" << m_lines[i]->straightnessFactor << std::endl;
//    }
}

Line** AbstractStrategy::findBestLine()
{
    Line* ret = NULL;
    Line* similar = NULL;

    sortLinesByStraightness();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        lockAllLines(false);
        ret = m_lines[i];

        lockSimilarLines(ret);

        //lockedCount();

        similar = findLineWithSameDirection(ret);

        if (ret != NULL && similar != NULL)
        {
            //std::cout << "RET " << ret->computeDirectionInDegrees() << " similar " << similar->computeDirectionInDegrees() << std::endl;
            std::cout << *ret << std::endl;
            std::cout << *similar << std::endl;
            setBestLine(ret, similar);
            writeLineInImage(ret, 255, 0, 0);
            writeLineInImage(similar, 0, 0, 255);
            break;
        }
        else
        {
            std::cout << "fail!" << std::endl;
        }
    }
    return m_bestLine;
}

void AbstractStrategy::lockSimilarLines(Line* input)
{
    Line* temp = NULL;

    if (input == NULL) return;

    for (unsigned int index = 0; index < m_lines.size(); index++)
    {
        temp = m_lines[index];

        if (temp->isClose(input) || temp->isInline(input))
        {
            temp->locked = true;
        }
    }
}

void AbstractStrategy::writeLineInImage(Line* line, int r, int g, int b)
{
    Vector2<int> linePoint;

    if (line == NULL) return;

    for (unsigned int i = 0; i < line->points.size(); i++)
    {
        linePoint = line->points[i];
        if (i < 15)
        {
            m_bmpImage->setPixelValue(linePoint.y, linePoint.x, 0, 255, 255);
        }
        else
        {
            m_bmpImage->setPixelValue(linePoint.y, linePoint.x, r, g, b);
        }
    }
}

void AbstractStrategy::lockAllLines(bool val)
{
    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        m_lines[i]->locked = val;
    }
}

void AbstractStrategy::lockedCount()
{
    unsigned int count = m_lines.size();
    unsigned int tmp = 0;
    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        if (m_lines[i]->locked)
        {
            tmp++;
        }
    }

    std::cout << "locked/count " << tmp << "/" << count << std::endl;
}

/** 
 * @param input
 * @return Line*
 */
Line* AbstractStrategy::findLineWithSameDirection(Line* input)
{
    if (input == NULL || m_lines.size() == 0) return NULL;

    double minDelta = 10000.0;
    double delta = 0;
    double direction = input->directionDegrees;
    Line* bestLine = NULL;

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        if (!m_lines[i]->locked)
        {
            double testedDirection = m_lines[i]->directionDegrees;

            delta = direction - testedDirection;
            delta = delta < 0 ? -delta : delta;

            //std::cout << "minDelta " << minDelta << " delta " << delta << " originDirection = " << direction << " testedDirection = " << testedDirection << std::endl;

            if (delta < minDelta)
            {
                if (delta < DIRECTION_DELTA)
                {
                    minDelta = delta;
                    std::cout << " change MIN " << minDelta << std::endl;
                    bestLine = m_lines[i];
                }
            }
        }
    }

    return bestLine;
}

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
        if (pixel->r > SELECTION_TRESHOLD || pixel->b > SELECTION_TRESHOLD || pixel->g > SELECTION_TRESHOLD)
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

Line* AbstractStrategy::getLongestLine()
{
    Line* longest = NULL;
    double maxLineSize = 0;
    double tempSize = 0;

    if (m_lines.size() > 0)
    {
        for (unsigned int i = 0; i < m_lines.size(); i++)
        {
            if (!m_lines[i]->locked)
            {
                tempSize = m_lines[i]->length;

                if (tempSize > maxLineSize)
                {
                    longest = m_lines[i];
                    maxLineSize = tempSize;
                }
            }
        }
        longest->locked = true;
    }
    return longest;
}

Line* AbstractStrategy::getStraightestLine()
{
    Line* straightest = NULL;
    float minStraightFactor = 100000;
    float tempStraightnessFactor = 0;

    if (m_lines.size() > 0)
    {
        for (unsigned int i = 0; i < m_lines.size(); i++)
        {
            if (!m_lines[i]->locked)
            {
                tempStraightnessFactor = m_lines[i]->straightnessFactor;

                if (tempStraightnessFactor < minStraightFactor)
                {
                    straightest = m_lines[i];
                    minStraightFactor = tempStraightnessFactor;
                }
            }
        }
        straightest->locked = true;
    }
    return straightest;
}
