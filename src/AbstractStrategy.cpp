#include "AbstractStrategy.h"
#include "Timer.h"
#include "DetectionParams.h"

AbstractStrategy::AbstractStrategy(DetectionSettings* settings)
: m_settings(settings)
{
    setBaseColor();
    m_bestLine = new BestLine();
}

AbstractStrategy::AbstractStrategy(BmpImage<float>* image, DetectionSettings* settings)
: m_workImage(image), m_settings(settings)
{
    setBaseColor();
    m_bestLine = new BestLine();
}

AbstractStrategy::~AbstractStrategy()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();
    SAFE_DELETE(m_bestLine);
}

void AbstractStrategy::setImages(BmpImage<float>* image, BmpImage<float>* colorImage)
{
    cleanUp();
    m_workImage = image;
    m_colorImage = colorImage;
}

void AbstractStrategy::cleanUp()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();
    m_bestLine->invalidate();
}

void AbstractStrategy::smooth()
{
    const double m = 1.0 / 9;
    double result;

    Pixel<float>* pixel = NULL;

    for (unsigned int y = 1; y < m_workImage->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_workImage->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_workImage->getPixel(y, x);

                result = m * m_workImage->getPixelChannelValue(y - 1, x - 1, ch) +
                        m * m_workImage->getPixelChannelValue(y - 1, x, ch) +
                        m * m_workImage->getPixelChannelValue(y - 1, x + 1, ch) +
                        m * m_workImage->getPixelChannelValue(y, x - 1, ch) +
                        m * m_workImage->getPixelChannelValue(y, x, ch) +
                        m * m_workImage->getPixelChannelValue(y, x + 1, ch) +
                        m * m_workImage->getPixelChannelValue(y + 1, x - 1, ch) +
                        m * m_workImage->getPixelChannelValue(y + 1, x, ch) +
                        m * m_workImage->getPixelChannelValue(y + 1, x + 1, ch);

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
            m_workImage->setPixelValue(y, x, pixel);
        }
    }
}

void AbstractStrategy::gaussianBlur()
{
    const double m = 1.0 / 16;
    double result;

    Pixel<float>* pixel = NULL;

    for (unsigned int y = 1; y < m_workImage->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_workImage->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_workImage->getPixel(y, x);

                result = m * m_workImage->getPixelChannelValue(y - 1, x - 1, ch) +
                        m * 2.0 * m_workImage->getPixelChannelValue(y - 1, x, ch) +
                        m * m_workImage->getPixelChannelValue(y - 1, x + 1, ch) +
                        m * 2.0 * m_workImage->getPixelChannelValue(y, x - 1, ch) +
                        m * 4.0 * m_workImage->getPixelChannelValue(y, x, ch) +
                        m * 2.0 * m_workImage->getPixelChannelValue(y, x + 1, ch) +
                        m * m_workImage->getPixelChannelValue(y + 1, x - 1, ch) +
                        m * 2.0 * m_workImage->getPixelChannelValue(y + 1, x, ch) +
                        m * m_workImage->getPixelChannelValue(y + 1, x + 1, ch);

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
            m_workImage->setPixelValue(y, x, pixel);
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

    for (unsigned int y = 1; y < m_workImage->getHeight() - 1; y++)
    {
        for (unsigned int x = 1; x < m_workImage->getWidth() - 1; x++)
        {
            for (int ch = 0; ch < 3; ch++)
            {
                pixel = m_workImage->getPixel(y, x);

                result =
                        m * -2.0 * m_workImage->getPixelChannelValue(y - 1, x, ch) +
                        m * -2.0 * m_workImage->getPixelChannelValue(y, x - 1, ch) +
                        m * 11.0 * m_workImage->getPixelChannelValue(y, x, ch) +
                        m * -2.0 * m_workImage->getPixelChannelValue(y, x + 1, ch) +
                        m * -2.0 * m_workImage->getPixelChannelValue(y + 1, x, ch);

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
            m_workImage->setPixelValue(y, x, pixel);
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

    for (unsigned int i = 0; i < m_workImage->getHeight(); ++i)
    {
        for (unsigned int j = 0; j < m_workImage->getWidth(); ++j)
        {
            pixel = m_workImage->getPixel(i, j);

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
        if (lines[i]->computeLength() < DetectionParams::lineLengthTreshold)
        {
            SAFE_DELETE(lines[i]);
        }
    }

    Line* best = Line::getMaxLengthLine(lines);
    //Line* best = Line::getStraightesstLine(lines);
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
        if (best->straightnessFactor < DetectionParams::straightnessTreshold)
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

    for (unsigned int i = 1; i < m_workImage->getHeight() - 1; ++i)
    {
        for (unsigned int j = 1; j < m_workImage->getWidth() - 1; ++j)
        {
            pixel = m_workImage->getPixel(i, j);

            if (pixel->r > DetectionParams::selectionTreshold
                    || pixel->b > DetectionParams::selectionTreshold
                    || pixel->g > DetectionParams::selectionTreshold)
            {
                lines[0] = findCorrectLine(1, 0, 0, 1, i, j);
                lines[1] = findCorrectLine(-1, 0, 0, 1, i, j);

                lines[2] = findCorrectLine(0, 1, 1, 0, i, j);
                lines[3] = findCorrectLine(0, -1, 1, 0, i, j);

                if (storeBestLine(lines))
                {
                    if(m_workImage->getWidth() / 15 < 1)
                        j++;
                    else
                        j += m_workImage->getWidth() / 15;
                    //break;
                }
            }
        }
    }
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

BestLine* AbstractStrategy::findBestLine()
{
    Line* ret = NULL;
    Line* similar = NULL;

    //sortLinesByStraightness();
    sortLinesByLength();

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        lockAllLines(false);
        ret = m_lines[i];

        lockSimilarLines(ret);

        //lockedCount();

        similar = findLineWithSameDirection(ret);

        if (ret != NULL && similar != NULL)
        {
            if (lineColorMatch(ret, similar))
            {
                std::cout << "RET " << ret->computeDirectionInDegrees() << " similar " << similar->computeDirectionInDegrees() << std::endl;
                std::cout << *ret << std::endl;
                std::cout << *similar << std::endl;
                writeLineInImage(ret, 255, 0, 0);
                writeLineInImage(similar, 0, 0, 255);
                m_bestLine->setLine(ret, similar);
            }
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
            m_workImage->setPixelValue(linePoint.y, linePoint.x, 0, 255, 255);
        }
        else
        {
            m_workImage->setPixelValue(linePoint.y, linePoint.x, r, g, b);
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

bool AbstractStrategy::lineColorMatch(Line* l1, Line* l2)
{        
    double halfDist1 = l2->getDistanceTo(l1->points.front()) / 2;
    double halfDist2 = l2->getDistanceTo(l1->points.back()) / 2;
    
    //TODO
    
    //check color between lines if matches
    return true;
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
                if (delta < DetectionParams::directionDeltaDegrees)
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

    while (posY > 2 && posX > 2 && posY < m_workImage->getHeight() - 2 && posX < m_workImage->getWidth() - 2)
    {
        posY += vectorY;
        posX += vectorX;

        pixel = m_workImage->getPixel(posY, posX);

        if (pixel->r > DetectionParams::selectionTreshold
                || pixel->b > DetectionParams::selectionTreshold
                || pixel->g > DetectionParams::selectionTreshold)
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
