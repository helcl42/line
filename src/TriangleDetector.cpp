#include "TriangleDetector.h"

TriangleDetector::TriangleDetector(DetectionColorItem* settings)
: ObjectDetector(settings)
{
    this->initDetectionParams();
    m_bestTriangle = new Triangle();
    m_tempLine = new Line();
}

TriangleDetector::TriangleDetector(Image<float>* image, Image<float>* colorImage)
: ObjectDetector(image, colorImage)
{
    this->initDetectionParams();
    m_bestTriangle = new Triangle();
    m_tempLine = new Line();
}

TriangleDetector::~TriangleDetector()
{
    for (unsigned int i = 0; i < m_triangles.size(); ++i)
    {
        SAFE_DELETE(m_triangles[i]);
    }
    m_triangles.clear();
    SAFE_DELETE(m_bestTriangle);
    SAFE_DELETE(m_tempLine);
}

void TriangleDetector::invalidate()
{
    for (unsigned int i = 0; i < m_triangles.size(); ++i)
    {
        SAFE_DELETE(m_triangles[i]);
    }
    m_triangles.clear();
    m_bestTriangle->invalidate();
    m_tempLine->deletePoints();
}

void TriangleDetector::setAngles(std::vector<float> angles)
{
    m_angles = angles;
}

void TriangleDetector::generateTriangles(unsigned int size)
{
    Triangle* triangle = NULL;    
    unsigned int offset = size / 4;

    for (unsigned int rotateAngle = 0; rotateAngle <= 90; rotateAngle += 3)
    {        
        triangle = generateTriangle(offset, offset, size, rotateAngle);
        m_triangles.push_back(triangle);        
    }
}

Triangle* TriangleDetector::generateTriangle(int x0, int y0, int size, float angle)
{
    Triangle* triangle = new Triangle();
    double sinAngle = sin(angle * M_PI / 180);
    double cosAngle = cos(angle * M_PI / 180);   
    int halfSize = size >> 1;           
    
    for (int i = y0 - halfSize; i < y0 + halfSize; i++)
    {
        for (int j = x0 - halfSize; j < x0 + halfSize; j++)
        {
            if (i == y0 - halfSize || i == y0 + halfSize - 1)
            {
                triangle->addPoint(Vector2<int>((j - x0) * cosAngle - (i - y0) * sinAngle, (j - x0) * sinAngle + (i - y0) * cosAngle));
            }
            else
            {
                if (j == x0 - halfSize || j == x0 + halfSize - 1)
                {
                    triangle->addPoint(Vector2<int>((j - x0) * cosAngle - (i - y0) * sinAngle, (j - x0) * sinAngle + (i - y0) * cosAngle));
                }
            }
        }
    }
    
    Line* line = triangle->getAt(0);
    for(int i = 0; i < line->getSize(); i++)
    {        
        line->points[i] = Vector2<int>(line->points[i].x + x0 + halfSize, line->points[i].y + y0 + halfSize);
    }    
    return triangle;
}

Triangle* TriangleDetector::findObject()
{
    //repaintSimilarColorPlaces();
    m_edgeFilter->setImage(m_workImage);
    m_edgeFilter->applyFilter(DetectionParams::colorTreshold);
    m_imageFilter->setImage(m_workImage);
    m_imageFilter->gaussianBlur();
    m_imageMap->setImage(m_workImage);
    return findBestTriangle();
}

bool TriangleDetector::rawTriangleFind(Triangle* triangle, unsigned int y, unsigned int x)
{    
    unsigned int ratio = 8, failCount = 0;
    double percentFail;    
    Line* triangleLine = triangle->getAt(0);
    unsigned int lineSize = triangleLine->getSize() / ratio;
    
    for (unsigned int k = 0; k < lineSize; k++)
    {
        Vector2<int> point = triangleLine->points[k];
        if (m_imageMap->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold) failCount++;        
    }    
    
    percentFail = (double) failCount / (double) lineSize;
    
    if(percentFail < 0.01) return true;        
    return false;
}

bool TriangleDetector::innerTriangleFind(Triangle* triangle, unsigned int y, unsigned int x)
{
    Line* triangleLine = triangle->getAt(0);
    unsigned int lineSize = triangleLine->getSize();
    unsigned int failCount = 0;
    double percentFail;
    //Line line;

    if(!rawTriangleFind(triangle, y, x)) return false;
    
    for (unsigned int k = 0; k < lineSize; k++)
    {
        Vector2<int> point = triangleLine->points[k];
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
            Vector2<int> point = triangleLine->points[k];
            m_tempLine->points.push_back(Vector2<int>(point.x + x, point.y + y));
        }
        m_bestTriangle->invalidate();
        m_bestTriangle->setAt(m_tempLine, 0);
        return true;
    }
    return false;
}

bool TriangleDetector::findTriangleInImagePart(unsigned int imagePart, unsigned int triangleSize)
{
    unsigned int baseHeight = m_imageMap->getHeight();
    unsigned int baseWidth = m_imageMap->getWidth();
    unsigned int offset = pow(2, 0.5) * triangleSize;

    Triangle* triangle = m_triangles[imagePart];

    std::vector<float>::reverse_iterator ri;
    for (ri = m_angles.rbegin(); ri != m_angles.rend(); ++ri)
    {
        triangle->projectByAngle(*ri);        

        for (unsigned int i = 0; i < baseHeight - offset - 1; i += 2)
        {
            for (unsigned int j = 0; j < baseWidth - 2 * offset; j += 2)
            {
                if (innerTriangleFind(triangle, i, j))
                {
                    return true;
                }                
            }
        }
    }
    return false;
}

Triangle* TriangleDetector::findBestTriangle()
{
    unsigned int triangleSize = m_imageMap->getHeight() / 3 - 1;

    while (triangleSize > m_imageMap->getHeight() / 5)
    {
        generateTriangles(triangleSize);

        for (unsigned int i = 0; i < m_triangles.size(); i++)
        {
            if (findTriangleInImagePart(i, triangleSize))
            {
                if (colorMatch())
                {
                    std::cout << *m_bestTriangle << std::endl;
                    writeLineInImage(*m_bestTriangle->getLines(), 255, 0, 0);
                    break;
                }
                else
                {
                    m_bestTriangle->invalidate();
                }
            }
        }
        invalidate();

        triangleSize -= 3;
    }
    return m_bestTriangle;
}

bool TriangleDetector::colorMatch(unsigned int failCount)
{
    Vector2<int>* ret;
    Pixel<float>* pixel;
    Line* line = m_bestTriangle->getAt(0);  
    unsigned int lenSixth = line->getSize() / 6;     
    unsigned int offset = 0;
    
    for (unsigned int i = 0; i < 2; i++, offset += 2); 
    {        
        ret = Vector2<int>::getPointBetween(line->points[lenSixth], line->points[offset * lenSixth]);

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        SAFE_DELETE(ret);
        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        {
            return false;
        }        
    }
    return true;
}

void TriangleDetector::initDetectionParams(unsigned int shrink)
{
    DetectionParams::selectionTreshold = 30;

    DetectionParams::maxPercentageError = 0.1;
}