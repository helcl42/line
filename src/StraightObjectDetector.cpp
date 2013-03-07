#include "StraightObjectDetector.h"


StraightObjectDetector::StraightObjectDetector(DetectionColorItem* settings)
: ObjectDetector(settings) {}

StraightObjectDetector::StraightObjectDetector(Image<float>* image, Image<float>* colorImage)
: ObjectDetector(image, colorImage) {}

StraightObjectDetector::~StraightObjectDetector()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();        
}

void StraightObjectDetector::writeLineInImage(Line* line, int r, int g, int b)
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


void StraightObjectDetector::lockSimilarLines(Line* input)
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


void StraightObjectDetector::lockAllLines(bool val)
{
    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        m_lines[i]->locked = val;
    }
}

void StraightObjectDetector::traverseImage()
{
    Pixel<float>* pixel = NULL;
    Line * lines[8];

    Vector2<int> vecs1[] = {
        Vector2<int>(0, 1),
        Vector2<int>(1, 1), Vector2<int>(-1, 1),
        Vector2<int>(1, 0)
    };

    Vector2<int> vecs2[] = {
        Vector2<int>(0, -1),
        Vector2<int>(1, -1), Vector2<int>(-1, -1),
        Vector2<int>(-1, 0)
    };

    Vector2<int> vecs3[] = {
        Vector2<int>(1, 0),
        Vector2<int>(1, 1), Vector2<int>(1, -1),
        Vector2<int>(0, 1)
    };

    Vector2<int> vecs4[] = {
        Vector2<int>(-1, 0),
        Vector2<int>(-1, 1), Vector2<int>(-1, -1),
        Vector2<int>(0, -1)
    };

    Vector2<int> vecs5[] = {
        Vector2<int>(1, 1),
        Vector2<int>(1, 0), Vector2<int>(0, 1),
        Vector2<int>(1, -1)
    };

    Vector2<int> vecs6[] = {
        Vector2<int>(1, -1),
        Vector2<int>(0, -1), Vector2<int>(1, 0),
        Vector2<int>(1, 1)
    };

    Vector2<int> vecs7[] = {
        Vector2<int>(-1, -1),
        Vector2<int>(0, -1), Vector2<int>(-1, 0),
        Vector2<int>(-1, 1)
    };

    Vector2<int> vecs8[] = {
        Vector2<int>(-1, 1),
        Vector2<int>(0, 1), Vector2<int>(-1, 0),
        Vector2<int>(1, 1)
    };

    for (unsigned int i = 1; i < m_workImage->getHeight() - 1; i += 2)
    {
        for (unsigned int j = 1; j < m_workImage->getWidth() - 1; j += 2)
        {
            pixel = m_workImage->getPixel(i, j);

            if (pixel->r > DetectionParams::selectionTreshold || pixel->b > DetectionParams::selectionTreshold || pixel->g > DetectionParams::selectionTreshold)
            {
                Vector2<int> index(j, i);
                lines[0] = findCorrectLine(vecs1, index);
                lines[1] = findCorrectLine(vecs2, index);
                lines[2] = findCorrectLine(vecs3, index);
                lines[3] = findCorrectLine(vecs4, index);

                lines[4] = findCorrectLine(vecs5, index);
                lines[5] = findCorrectLine(vecs6, index);
                lines[6] = findCorrectLine(vecs7, index);
                lines[7] = findCorrectLine(vecs8, index);

                if (storeBestLine(lines)) j++;                
            }
        }
    }
}

bool StraightObjectDetector::storeBestLine(Line** lines)
{
    for (int i = 0; i < 8; i++)
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
        for (int i = 0; i < 8; i++)
        {
            if (best != lines[i])
            {
                SAFE_DELETE(lines[i]);
            }
        }

        best->computeProperties();
        if (best->straightnessFactor < DetectionParams::straightnessTreshold)
        {            
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

Line* StraightObjectDetector::findCorrectLine(Vector2<int>* vecs, Vector2<int> pos)
{
    Pixel<float>* pixel = NULL;
    Line* line = new Line();
    unsigned int countOfFails = 0;

    line->points.push_back(Vector2<int>(pos));

    while (pos.y > 2 && pos.x > 2 && pos.y < (int) m_workImage->getHeight() - 2 && pos.x < (int) m_workImage->getWidth() - 2)
    {
        pos += vecs[countOfFails];

        pixel = m_workImage->getPixel(pos.y, pos.x);

        if (pixel->r > DetectionParams::selectionTreshold || pixel->b > DetectionParams::selectionTreshold || pixel->g > DetectionParams::selectionTreshold)
        {
            countOfFails = 0;

            line->points.push_back(pos);
        }
        else
        {
            pos -= vecs[countOfFails];

            countOfFails++;

            if (countOfFails > DetectionParams::countOfDirections)
            {
                break;
            }
        }
    }
    return line;
}


void StraightObjectDetector::sortLinesByLength()
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


void StraightObjectDetector::sortLinesByStraightness()
{
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
}


/** 
 * @param input
 * @return Line*
 */
Line* StraightObjectDetector::findLineWithDirection(Line* input, float angle)
{
    if (input == NULL || m_lines.size() == 0) return NULL;

    double minDelta = 10000.0;
    double delta;
    double testedDirection;
    Line* bestLine = NULL;

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        if (m_lines[i]->locked) continue;        

        testedDirection = m_lines[i]->directionDegrees;

        delta = input->directionDegrees - testedDirection + angle;
        delta = delta < 0 ? -delta : delta;

        if (delta < minDelta)
        {
            if (delta < DetectionParams::directionDeltaDegrees)
            {
                minDelta = delta;
                bestLine = m_lines[i];
            }
        }
    }

    return bestLine;
}

Line* StraightObjectDetector::getLongestLine()
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


Line* StraightObjectDetector::getStraightestLine()
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