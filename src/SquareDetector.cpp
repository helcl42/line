#include "SquareDetector.h"

SquareDetector::SquareDetector(DetectionColorItem* settings)
: ObjectDetector(settings)
{
    this->initDetectionParams();
    m_bestSquare = new Square();
    m_tempLine = new Line();
}

SquareDetector::SquareDetector(Image<float>* image, Image<float>* colorImage)
: ObjectDetector(image, colorImage)
{
    this->initDetectionParams();
    m_bestSquare = new Square();
    m_tempLine = new Line();
}

SquareDetector::~SquareDetector()
{
    for (unsigned int i = 0; i < m_squares.size(); ++i)
    {
        SAFE_DELETE(m_squares[i]);
    }
    m_squares.clear();
    SAFE_DELETE(m_bestSquare);
    SAFE_DELETE(m_tempLine);
}

void SquareDetector::invalidate()
{
    for (unsigned int i = 0; i < m_squares.size(); ++i)
    {
        SAFE_DELETE(m_squares[i]);
    }
    m_squares.clear();
    m_bestSquare->invalidate();
    m_tempLine->deletePoints();
}

void SquareDetector::setAngles(std::vector<float> angles)
{
    m_angles = angles;
}

void SquareDetector::generateSquares(unsigned int size)
{
    Square* square = NULL;    
    unsigned int offset = size / 4;

    for (unsigned int rotateAngle = 0; rotateAngle <= 90; rotateAngle += 3)
    {        
        square = generateSquare(offset, offset, size, rotateAngle);
        m_squares.push_back(square);        
    }
}

/**
 * POINT rotate_point(float cx,float cy,float angle,POINT p)
{
  float s = sin(angle);
  float c = cos(angle);

  // translate point back to origin:
  p.x -= cx;
  p.y -= cy;

  // rotate point
  float xnew = p.x * c - p.y * s;
  float ynew = p.x * s + p.y * c;

  // translate point back:
  p.x = xnew + cx;
  p.y = ynew + cy;
} 
 */
Square* SquareDetector::generateSquare(int x0, int y0, int size, float angle)
{
    Square* square = new Square();
    double sinAngle = sin(angle * M_PI / 180);
    double cosAngle = cos(angle * M_PI / 180);   
    int halfSize = size >> 1;           
    
    for (int i = y0 - halfSize; i < y0 + halfSize; i++)
    {
        for (int j = x0 - halfSize; j < x0 + halfSize; j++)
        {
            if (i == y0 - halfSize || i == y0 + halfSize - 1)
            {
                square->addPoint(Vector2<int>((j - x0) * cosAngle - (i - y0) * sinAngle, (j - x0) * sinAngle + (i - y0) * cosAngle));
            }
            else
            {
                if (j == x0 - halfSize || j == x0 + halfSize - 1)
                {
                    square->addPoint(Vector2<int>((j - x0) * cosAngle - (i - y0) * sinAngle, (j - x0) * sinAngle + (i - y0) * cosAngle));
                }
            }
        }
    }
    
    Line* line = square->getAt(0);
    for(int i = 0; i < line->getSize(); i++)
    {        
        line->points[i] = Vector2<int>(line->points[i].x + x0 + halfSize, line->points[i].y + y0 + halfSize);
    }    
    return square;
}

Square* SquareDetector::findObject()
{
    //repaintSimilarColorPlaces();
    m_edgeFilter->setImage(m_workImage);
    m_edgeFilter->applyFilter(DetectionParams::colorTreshold);
    m_imageFilter->setImage(m_workImage);
    m_imageFilter->gaussianBlur();
    m_imageMap->setImage(m_workImage);
    return findBestSquare();
}

bool SquareDetector::innerSquareFind(Square* square, unsigned int y, unsigned int x)
{
    Line* squareLine = square->getAt(0);
    unsigned int lineSize = squareLine->getSize();
    unsigned int failCount = 0;
    double percentFail;
    //Line line;

    for (unsigned int k = 0; k < lineSize; k++)
    {
        Vector2<int> point = squareLine->points[k];
        if (m_imageMap->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold)
        {
            failCount++;
        }
        //line.points.push_back(Vector2<int>(point.x + x, point.y + y));
    }
    //writeLineInImage(&line, 0, 255, 0);
    percentFail = (double) failCount / (double) lineSize;

    if (percentFail < DetectionParams::maxPercentageError)
    {
        std::cout << "failCount = " << failCount << " size = " << lineSize << " fail = " << percentFail << "%" << std::endl;
        m_tempLine->deletePoints();
        for (unsigned int k = 0; k < lineSize; k++)
        {
            Vector2<int> point = squareLine->points[k];
            m_tempLine->points.push_back(Vector2<int>(point.x + x, point.y + y));
        }
        m_bestSquare->invalidate();
        m_bestSquare->setAt(m_tempLine, 0);
        return true;
    }
    return false;
}

bool SquareDetector::findSquareInImagePart(unsigned int imagePart, unsigned int squareSize)
{
    unsigned int baseHeight = m_imageMap->getHeight();
    unsigned int baseWidth = m_imageMap->getWidth();
    unsigned int offset = pow(2, 0.5) * squareSize;

    Square* square = m_squares[imagePart];

    std::vector<float>::reverse_iterator ri;
    for (ri = m_angles.rbegin(); ri != m_angles.rend(); ++ri)
    {
        square->projectByAngle(*ri);        

        for (unsigned int i = 0; i < baseHeight - offset - 1; i += 2)
        {
            for (unsigned int j = 0; j < baseWidth - 2 * offset; j += 2)
            {
                if (innerSquareFind(square, i, j))
                {
                    return true;
                }                
            }
        }
    }
    return false;
}

Square* SquareDetector::findBestSquare()
{
    unsigned int squareSize = m_imageMap->getHeight() / 3 - 1;

    while (squareSize > m_imageMap->getHeight() / 5)
    {
        generateSquares(squareSize);

        for (unsigned int i = 0; i < m_squares.size(); i++)
        {
            if (findSquareInImagePart(i, squareSize))
            {
                if (colorMatch())
                {
                    std::cout << *m_bestSquare << std::endl;
                    writeLineInImage(*m_bestSquare->getLines(), 255, 0, 0);
                    break;
                }
                else
                {
                    m_bestSquare->invalidate();
                }
            }
        }
        invalidate();

        squareSize -= 3;
    }
    return m_bestSquare;
}

bool SquareDetector::colorMatch(unsigned int failCount)
{
    Vector2<int>* ret;
    Pixel<float>* pixel;
    Line* line = m_bestSquare->getAt(0);  
    unsigned int lenFourth = line->getSize() / 4;
        
    for (unsigned int i = 0; i < 4; i += lenFourth)
    {
        ret = Vector2<int>::getPointBetween(line->points[i * lenFourth], line->points[(i + 2) * lenFourth]);

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        SAFE_DELETE(ret);
        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        {
            return false;
        }        
    }
    return true;
}

void SquareDetector::initDetectionParams(unsigned int shrink)
{
    DetectionParams::selectionTreshold = 30;

    DetectionParams::maxPercentageError = 0.1;
}