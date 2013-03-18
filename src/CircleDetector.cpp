#include "CircleDetector.h"
#include "CameraService.h"

CircleDetector::CircleDetector(DetectionColorItem* settings)
: ObjectDetector(settings)
{
    this->initDetectionParams();
    m_bestCircle = new Circle();
    m_tempLine = new Line();
}

CircleDetector::CircleDetector(Image<float>* image, Image<float>* colorImage)
: ObjectDetector(image, colorImage)
{
    this->initDetectionParams();
    m_bestCircle = new Circle();
    m_tempLine = new Line();
}

CircleDetector::~CircleDetector()
{
    for (unsigned int i = 0; i < m_ellipses.size(); ++i)
    {
        SAFE_DELETE(m_ellipses[i]);
    }
    m_ellipses.clear();
    SAFE_DELETE(m_bestCircle);
    SAFE_DELETE(m_tempLine);
}

void CircleDetector::invalidate()
{
    for (unsigned int i = 0; i < m_ellipses.size(); ++i)
    {
        SAFE_DELETE(m_ellipses[i]);
    }
    m_ellipses.clear();
    m_bestCircle->invalidate();
    m_tempLine->deletePoints();
}

void CircleDetector::writeLineInImage(Line* line, int r, int g, int b)
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

void CircleDetector::setAngles(std::vector<float> angles)
{
    m_angles = angles;
}

void CircleDetector::generateEllipses(unsigned int size)
{
    Line* ellipse = NULL;
    unsigned int imagePart = m_imageMap->getHeight() / 4;
    unsigned int x = size;

    std::vector<float>::reverse_iterator ri;
    for (ri = m_angles.rbegin(); ri != m_angles.rend(); ++ri)
    {
        ellipse = generateEllipse(x, size, size, *ri);
        //writeLineInImage(ellipse, 0, 0, 255);
        m_ellipses.push_back(ellipse);
        //x += (imagePart - size);
    }
}

/**   
 * ddF_x == 2 * x + 1;
 * ddF_y == -2 * y;
 * f == x*x + y*y - radius*radius + 2*x - y + 1;
 */
Line* CircleDetector::generateEllipse(int x0, int y0, int radius, float angle)
{
    Line* circle = new Line();
    int f = 1 - radius;
    int ddFx = 1;
    int ddFy = -2 * radius;
    int x = 0;
    int y = radius;
    float ellipse = sin(angle * M_PI / 180);

    circle->points.push_back(Vector2<int>(y0 + radius, x0));
    circle->points.push_back(Vector2<int>(y0 - radius, x0));
    circle->points.push_back(Vector2<int>(y0, x0 + radius * ellipse));
    circle->points.push_back(Vector2<int>(y0, x0 - radius * ellipse));

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddFy += 2;
            f += ddFy;
        }

        x++;
        ddFx += 2;
        f += ddFx;

        circle->points.push_back(Vector2<int>(y0 + y, x0 + x * ellipse));
        circle->points.push_back(Vector2<int>(y0 + y, x0 - x * ellipse));
        circle->points.push_back(Vector2<int>(y0 - y, x0 + x * ellipse));
        circle->points.push_back(Vector2<int>(y0 - y, x0 - x * ellipse));

        circle->points.push_back(Vector2<int>(y0 + x, x0 + y * ellipse));
        circle->points.push_back(Vector2<int>(y0 + x, x0 - y * ellipse));
        circle->points.push_back(Vector2<int>(y0 - x, x0 + y * ellipse));
        circle->points.push_back(Vector2<int>(y0 - x, x0 - y * ellipse));
    }
    return circle;
}

Circle* CircleDetector::findObject()
{
    //repaintSimilarColorPlaces();
    m_edgeFilter->setImage(m_workImage);
    m_edgeFilter->applyFilter(DetectionParams::colorTreshold);
    m_imageFilter->setImage(m_workImage);
    m_imageFilter->gaussianBlur();
    m_imageMap->setImage(m_workImage);
    return findBestCircle();
}

bool CircleDetector::innerEllipseFind(Line* ellipse, unsigned int y, unsigned int x)
{
    unsigned int failCount = 0;
    double percentFail;
    //Line line;

    for (unsigned int k = 0; k < ellipse->points.size(); k++)
    {
        Vector2<int> point = ellipse->points[k];
        if (m_imageMap->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold)
        {
            failCount++;
        }
        //line.points.push_back(Vector2<int>(point.x + x, point.y + y));
    }
    //writeLineInImage(&line, 0, 255, 0);
    percentFail = (double) failCount / (double) ellipse->points.size();

    if (percentFail < 0.05)
    {
        std::cout << "failCount = " << failCount << " size = " << ellipse->points.size() << " fail = " << percentFail << "%" << std::endl;        
        m_tempLine->deletePoints();        
        for (unsigned int k = 0; k < ellipse->points.size(); k++)
        {
            Vector2<int> point = ellipse->points[k];            
            m_tempLine->points.push_back(Vector2<int>(point.x + x, point.y + y));
        }
        m_bestCircle->setAt(m_tempLine, 0);
        return true;
    }
    return false;
}

bool CircleDetector::findEllipseInImagePart(unsigned int imagePart, unsigned int ellipseSize)
{
    Line* ellipse = m_ellipses[imagePart];
    unsigned int baseHeight = m_imageMap->getHeight() / 2;
    //unsigned int ellipseHeight = ellipseSize * sin(m_angles[imagePart] * M_PI / 180);   

    for (unsigned int i = 0; i < baseHeight; i += 1)
    {
        for (unsigned int j = 0; j < m_imageMap->getWidth() - 2 * ellipseSize; j += 1)
        {
            if (innerEllipseFind(ellipse, i, j))
            {
                return true;
            }
        }
    }
    return false;
}

Circle* CircleDetector::findBestCircle()
{
    unsigned int ellipseSize = m_imageMap->getHeight() / 4 - 1;

    while (ellipseSize > 8)
    {
        generateEllipses(ellipseSize);
        for (unsigned int i = 0; i < m_ellipses.size(); i++)
        {            
            if (findEllipseInImagePart(i, ellipseSize))
            {
                if (colorMatch())
                {
                    std::cout << *m_bestCircle << std::endl;
                    writeLineInImage(*m_bestCircle->getLines(), 255, 0, 0);
                    break;
                }
            }
        }
        invalidate();
        ellipseSize -= 1;
    }
    return m_bestCircle;
}

bool CircleDetector::colorMatch(unsigned int failCount)
{
    Vector2<int>* ret;
    Pixel<float>* pixel;
    Line* line = *m_bestCircle->getLines();
        
    for (unsigned int i = 0; i < 4; i += 2)
    {
        ret = Vector2<int>::getPointBetween(line->points[i], line->points[i + 1]);

        pixel = m_colorImage->getPixel(ret->y, ret->x);

        SAFE_DELETE(ret);
        if (!pixel->hasSimilarColor(&m_settings->color, DetectionParams::colorTolerance))
        {
            return false;
        }        
    }
    return true;
}

void CircleDetector::initDetectionParams(unsigned int shrink)
{
    DetectionParams::selectionTreshold = 30;
}

