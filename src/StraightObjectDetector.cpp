#include "StraightObjectDetector.h"

StraightObjectDetector::StraightObjectDetector(DetectionColorItem* settings)
: AbstractDetector(settings)
{
}

StraightObjectDetector::StraightObjectDetector(ImageMap<float>* image, Image<float>* colorImage)
: AbstractDetector(image, colorImage)
{
}

StraightObjectDetector::~StraightObjectDetector()
{
    for (unsigned int i = 0; i < m_lines.size(); ++i)
    {
        SAFE_DELETE(m_lines[i]);
    }
    m_lines.clear();
}

void StraightObjectDetector::lockSimilarLines(Polygon<int>* input, unsigned int crossCount)
{
    Polygon<int>* temp = NULL;

    if (input == NULL) return;

    for (unsigned int index = 0; index < m_lines.size(); index++)
    {
        temp = m_lines[index];

        if (temp->isClose(input, crossCount) || temp->isInline(input))
        {
            temp->setLocked(true);
        }
    }
}

void StraightObjectDetector::lockAllLines(bool val)
{
    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        m_lines[i]->setLocked(val);
    }
}

void StraightObjectDetector::traverseImage()
{
    unsigned int value;
    Polygon<int>* lines[8];

    Vector2<int> vecs1[] = {
        Vector2<int>(0, 1),
        Vector2<int>(1, 1), Vector2<int>(-1, 1),
        Vector2<int>(1, 0), Vector2<int>(-1, -1)
    };

    Vector2<int> vecs2[] = {
        Vector2<int>(0, -1),
        Vector2<int>(1, -1), Vector2<int>(-1, -1),
        Vector2<int>(-1, 0), Vector2<int>(-1, 1)
    };

    Vector2<int> vecs3[] = {
        Vector2<int>(1, 0),
        Vector2<int>(1, 1), Vector2<int>(1, -1),
        Vector2<int>(0, 1), Vector2<int>(-1, 1)
    };

    Vector2<int> vecs4[] = {
        Vector2<int>(-1, 0),
        Vector2<int>(-1, 1), Vector2<int>(-1, -1),
        Vector2<int>(0, -1), Vector2<int>(-1, -1)
    };

    Vector2<int> vecs5[] = {
        Vector2<int>(1, 1),
        Vector2<int>(1, 0), Vector2<int>(0, 1),
        Vector2<int>(1, -1), Vector2<int>(0, -1)
    };

    Vector2<int> vecs6[] = {
        Vector2<int>(1, -1),
        Vector2<int>(0, -1), Vector2<int>(1, 0),
        Vector2<int>(1, 1), Vector2<int>(0, 1)
    };

    Vector2<int> vecs7[] = {
        Vector2<int>(-1, -1),
        Vector2<int>(0, -1), Vector2<int>(-1, 0),
        Vector2<int>(-1, 1), Vector2<int>(0, 1)
    };

    Vector2<int> vecs8[] = {
        Vector2<int>(-1, 1),
        Vector2<int>(0, 1), Vector2<int>(-1, 0),
        Vector2<int>(1, 1), Vector2<int>(1, 0)
    };

    for (unsigned int i = 1; i < m_workImage->getHeight() - 1; i += 2)
    {
        for (unsigned int j = 1; j < m_workImage->getWidth() - 1; j += 2)
        {
            value = m_workImage->getValueAt(i, j);

            if (value > DetectionParams::selectionTreshold)
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

bool StraightObjectDetector::storeBestLine(Polygon<int>** lines)
{
    for (int i = 0; i < 8; i++)
    {
        if (lines[i]->computeLength() <= DetectionParams::minLineLengthTreshold
                || lines[i]->computeLength() >= DetectionParams::maxLineLengthTreshold)
        {
            SAFE_DELETE(lines[i]);
        }
    }

    Polygon<int>* best = Polygon<int>::getMaxLengthLine(lines);
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
        if (best->getStraightnessFactor() <= DetectionParams::maxStraightnessTreshold
                && best->getStraightnessFactor() >= DetectionParams::minStraightnessTreshold)
        {
            writeLineInImageMap(best, 255);
            m_lines.push_back(best);
            return true;
        }
        else
        {
            //std::cout << "FAIL factor = " << best->straightnessFactor << " len = " << best->length << std::endl;
            SAFE_DELETE(best);
            return false;
        }
    }
    else
    {
        return false;
    }
}

Polygon<int>* StraightObjectDetector::findCorrectLine(Vector2<int>* vecs, Vector2<int> pos)
{
    unsigned int value;
    unsigned int countOfFails = 0;

    Polygon<int>* line = new Polygon<int>();

    line->addPoint(Vector2<int>(pos));

    while (pos.y > 12 && pos.x > 12 && pos.y < (int) m_workImage->getHeight() - 12 && pos.x < (int) m_workImage->getWidth() - 12)
    {
        pos += vecs[countOfFails];

        value = m_workImage->getValueAt(pos.y, pos.x);

        if (value > DetectionParams::selectionTreshold)
        {
            countOfFails = 0;

            line->addPoint(pos);
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

void StraightObjectDetector::sortLinesByLength(bool reverse)
{
    Polygon<int>* temp = NULL;
    int j;

    for (unsigned int i = 1; i < m_lines.size(); i++)
    {
        temp = m_lines[i];
        for (j = i - 1; j >= 0; j--)
        {
            if (reverse)
            {
                if (m_lines[j]->getLength() > temp->getLength())
                {
                    break;
                }
            }
            else
            {
                if (m_lines[j]->getLength() < temp->getLength())
                {
                    break;
                }
            }
            m_lines[j + 1] = m_lines[j];
        }
        m_lines[j + 1] = temp;
    }
}

void StraightObjectDetector::sortLinesByStraightness(bool reverse)
{
    Polygon<int>* temp = NULL;
    int j;

    for (unsigned int i = 1; i < m_lines.size(); i++)
    {
        temp = m_lines[i];
        for (j = i - 1; j >= 0; j--)
        {
            if (reverse)
            {
                if (m_lines[j]->getStraightnessFactor() > temp->getStraightnessFactor())
                {
                    break;
                }
            }
            else
            {
                if (m_lines[j]->getStraightnessFactor() < temp->getStraightnessFactor())
                {
                    break;
                }
            }
            m_lines[j + 1] = m_lines[j];
        }
        m_lines[j + 1] = temp;
    }
}

Polygon<int>* StraightObjectDetector::findLineWithDirection(Polygon<int>* input, float angle)
{
    if (input == NULL || m_lines.size() == 0) return NULL;
    
    double delta, testedDirection, inputDirection;
    Polygon<int>* bestLine = NULL;
    Polygon<int>* tempLine = NULL;
    unsigned int countOfFails, step, position, ratio = 8;
    bool fromBegin = true;

    for (unsigned int i = 0; i < m_lines.size(); i++)
    {
        tempLine = m_lines[i];
        if (tempLine->isLocked()) continue;

        countOfFails = 0;
        position = 0;

        if (input->getSize() > tempLine->getSize())
        {
            step = tempLine->getSize() / ratio;
        }
        else
        {
            step = input->getSize() / ratio;
        }

        if (input->isBeginNearer(tempLine))
        {
            fromBegin = true;
        }
        else
        {
            fromBegin = false;
        }

        for (unsigned int j = 0; j < ratio; j++, position += step)
        {                        
            testedDirection = tempLine->computeDirectionDegreesTo(position, fromBegin);

            inputDirection = input->computeDirectionDegreesTo(position);

            delta = inputDirection - testedDirection + angle;

            if (delta < 0.0) delta = -delta;

            //std::cout << "testDirection = " << testedDirection << "    }input = " << input->directionDegrees << std::endl;           
            
            if(delta > DetectionParams::directionDeltaDegrees) 
            {
                countOfFails++;              
            }            
        }

        if (countOfFails == 0)
        {            
             bestLine = m_lines[i];         
        }
    }

    return bestLine;
}

//Polygon<int>* StraightObjectDetector::findLineWithDirection(Polygon<int>* input, float angle)
//{
//    if (input == NULL || m_lines.size() == 0) return NULL;
//
//    double minDelta = MAX_VALUE;
//    double delta;
//    double testedDirection;
//    Polygon<int>* bestLine = NULL;
//
//    for (unsigned int i = 0; i < m_lines.size(); i++)
//    {
//        if (m_lines[i]->isLocked()) continue;
//
//        testedDirection = m_lines[i]->getDirectionDegrees();
//
//        delta = input->getDirectionDegrees() - testedDirection + angle;
//        
//        if(delta < 0.0) delta = -delta;
//        
//        //std::cout << "testDirection = " << testedDirection << " input = " << input->directionDegrees << std::endl;
//
//        if (delta < minDelta)
//        {
//            if (delta < DetectionParams::directionDeltaDegrees)
//            {
//                minDelta = delta;
//                bestLine = m_lines[i];
//            }
//        }
//    }
//
//    return bestLine;
//}

Polygon<int>* StraightObjectDetector::getLongestLine()
{
    Polygon<int>* longest = NULL;
    double maxLineSize = 0;
    double tempSize = 0;

    if (m_lines.size() > 0)
    {
        for (unsigned int i = 0; i < m_lines.size(); i++)
        {
            if (!m_lines[i]->isLocked())
            {
                tempSize = m_lines[i]->getLength();

                if (tempSize > maxLineSize)
                {
                    longest = m_lines[i];
                    maxLineSize = tempSize;
                }
            }
        }
        longest->setLocked(true);
    }
    return longest;
}

Polygon<int>* StraightObjectDetector::getStraightestLine()
{
    Polygon<int>* straightest = NULL;
    float minStraightFactor = MAX_VALUE;
    float tempStraightnessFactor = 0;

    if (m_lines.size() > 0)
    {
        for (unsigned int i = 0; i < m_lines.size(); i++)
        {
            if (!m_lines[i]->isLocked())
            {
                tempStraightnessFactor = m_lines[i]->getStraightnessFactor();

                if (tempStraightnessFactor < minStraightFactor)
                {
                    straightest = m_lines[i];
                    minStraightFactor = tempStraightnessFactor;
                }
            }
        }
        straightest->setLocked(true);
    }
    return straightest;
}
