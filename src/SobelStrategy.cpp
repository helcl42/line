#include "SobelStrategy.h"

Line* SobelStrategy::detectLine()
{
    replaintSimilarColorPlaces(COLOR_TOLERANCE);
    blur();
    //m_bmpImage->shrinkImage(4);
    sobel();          
    return traverseImage();    
}

Line* SobelStrategy::traverseImage()
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
            if (pixel->r > 150 || pixel->b > 150 || pixel->g > 150)
            {
                line = findCorrectLine(1, 0, i, j);
                line2 = findCorrectLine(-1, 0, i, j);

                //if (line != NULL && line2 != NULL)
                {
                    unsigned int size1 = line->points.size();
                    unsigned int size2 = line2->points.size();
                    if (size1 > size2)
                    {
                        if (size1 > LINE_LENGTH_TRESHOLD)
                        {
                            std::cout << "found line" << std::endl;
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
                            std::cout << "found line" << std::endl;
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

Line* SobelStrategy::findCorrectLine(int vecY, int vecX, unsigned int posY, unsigned int posX)
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
        if (pixel->b > 150)
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

void SobelStrategy::sobel()
{
    double min = 1.0;
    double max = 0.0;

    unsigned int imageHeight = m_bmpImage->getHeight();
    unsigned int imageWidth = m_bmpImage->getWidth();

    double* buffer = new double[imageHeight * imageWidth];

    for (unsigned int i = 1; i < imageHeight - 1; i += 1)
    {
        for (unsigned int j = 1; j < imageWidth - 1; j += 1)
        {
            double gx =
                    1.0 * Pixel<float>::colourDifference(m_bmpImage->getPixel(i - 1, j - 1),
                    m_bmpImage->getPixel(i + 1, j - 1)) +
                    2.0 * Pixel<float>::colourDifference(m_bmpImage->getPixel(i - 1, j),
                    m_bmpImage->getPixel(i + 1, j)) +
                    1.0 * Pixel<float>::colourDifference(m_bmpImage->getPixel(i - 1, j + 1),
                    m_bmpImage->getPixel(i + 1, j + 1));

            double gy =
                    1.0 * Pixel<float>::colourDifference(m_bmpImage->getPixel(i - 1, j - 1),
                    m_bmpImage->getPixel(i - 1, j + 1)) +
                    2.0 * Pixel<float>::colourDifference(m_bmpImage->getPixel(i, j - 1),
                    m_bmpImage->getPixel(i, j + 1)) +
                    1.0 * Pixel<float>::colourDifference(m_bmpImage->getPixel(i + 1, j - 1),
                    m_bmpImage->getPixel(i + 1, j + 1));

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

            if (val > COLOR_TRESHOLD)
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

            m_bmpImage->setPixelValue(y, x, &pixel);
        }
    }
    SAFE_DELETE_ARRAY(buffer);
}

Line* SobelStrategy::getLongestLine(std::vector<Line*>& lines)
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


