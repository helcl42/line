#include "SvgObjectDetector.h"
#include "Shapes/GeneralObject.h"

SvgObjectDetector::SvgObjectDetector(std::vector<DetectedObject*>& shapes, DetectionColorItem* settings)
: ObjectDetector(settings), m_shapes(shapes)
{
    this->initDetectionParams();
    m_bestMatch = new GeneralObject();
    m_tempLine = new Line();
}

SvgObjectDetector::SvgObjectDetector(std::vector<DetectedObject*>& shapes, Image<float>* image, Image<float>* colorImage)
: ObjectDetector(image, colorImage), m_shapes(shapes)
{
    this->initDetectionParams();    
    m_bestMatch = new GeneralObject();
    m_tempLine = new Line();
}

SvgObjectDetector::~SvgObjectDetector()
{    
    m_shapes.clear();
    SAFE_DELETE(m_bestMatch);
    SAFE_DELETE(m_tempLine);
}

void SvgObjectDetector::invalidate()
{    
    m_tempLine->deletePoints();
}

void SvgObjectDetector::setAngles(std::vector<float> angles)
{
    m_angles = angles;
}

void SvgObjectDetector::generateShapes(unsigned int shapeIndex, unsigned int size)
{            
    DetectedObject* tempShapePtr;    
    
    for (unsigned int rotateAngle = 0; rotateAngle <= 90; rotateAngle += 5)
    {                 
        tempShapePtr = new GeneralObject(new Line(m_shapes[shapeIndex]->getPolygon()));
        tempShapePtr->rescaleToSize(size);
        tempShapePtr->rotateByAngle(rotateAngle);        
        m_detectedShapes.push_back(tempShapePtr);        
    }
}

DetectedObject* SvgObjectDetector::findObject()
{
    //repaintSimilarColorPlaces();
    m_edgeFilter->setImage(m_workImage);
    m_edgeFilter->applyFilter(DetectionParams::colorTreshold);
    m_imageFilter->setImage(m_workImage);
    m_imageFilter->gaussianBlur();
    m_imageMap->setImage(m_workImage);
    return findBestShape();
}

bool SvgObjectDetector::rawShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{    
    unsigned int ratio = 8, failCount = 0;
    double percentFail;    
    Line* squareLine = shape->getPolygon();
    unsigned int lineSize = squareLine->getSize() / ratio;
    
    for (unsigned int k = 0; k < lineSize; k++)
    {
        Vector2<int> point = squareLine->points[k];
        if (m_imageMap->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold) failCount++;        
    }    
    
    percentFail = (double) failCount / (double) lineSize;
    
    if(percentFail < 0.01) return true;        
    return false;
}

bool SvgObjectDetector::innerShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    Line* squareLine = shape->getPolygon();
    unsigned int lineSize = squareLine->getSize();
    unsigned int failCount = 0;
    double percentFail;
    Line line;
    
    if(!rawShapeFind(shape, y, x)) return false;
    
    for (unsigned int k = 0; k < lineSize; k++)
    {
        Vector2<int> point = squareLine->points[k];
        if (m_imageMap->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold)
        {
            failCount++;
        }
        line.points.push_back(Vector2<int>(point.x + x, point.y + y));
    }
    writeLineInImage(&line, 0, 255, 0);    
    
    return false;
    percentFail = (double) failCount / (double) lineSize;

    if (percentFail < DetectionParams::maxPercentageError)
    {
        Vector2<int>* point = NULL;
        std::cout << "failCount = " << failCount << " size = " << lineSize << " fail = " << percentFail << "%" << std::endl;
        m_tempLine->deletePoints();
        for (unsigned int k = 0; k < lineSize; k++)
        {
            point = squareLine->getPointPtr(k);
            m_tempLine->setPoint(point->x + x, point->y + y, k);
        }        
        invalidate();
        return true;
    }
    return false;
}

bool SvgObjectDetector::findShapeInImagePart(DetectedObject* shape)
{
    unsigned int baseHeight = m_imageMap->getHeight();
    unsigned int baseWidth = m_imageMap->getWidth();
    unsigned int offsetX = shape->getWidth();
    unsigned int offsetY = shape->getHeight();

    std::vector<float>::reverse_iterator anglesIterator;
    for (anglesIterator = m_angles.rbegin(); anglesIterator != m_angles.rend(); ++anglesIterator)
    {
        shape->viewByAngle(*anglesIterator);

        for (unsigned int i = 0; i < baseHeight - offsetY - 1; i += 4)
        {
            for (unsigned int j = 0; j < baseWidth - offsetX; j += 4)
            {
                if (innerShapeFind(shape, i, j))
                {
                    return true;
                }                
            }
        }
    }
    return false;
}

DetectedObject* SvgObjectDetector::findBestShape()
{    
    unsigned int shapeIndex = 0;
    unsigned int shapeSize = m_imageMap->getHeight() / 3 - 1;        
    
    while (shapeSize > m_imageMap->getHeight() / 5)
    {
        generateShapes(shapeIndex, shapeSize);

        for (unsigned int i = 0; i < m_detectedShapes.size(); i++)
        {
            if (findShapeInImagePart(m_detectedShapes[i]))
            {
//                if (colorMatch())
//                {
//                    //std::cout << *m_bestMatch << std::endl;                    
//                    //writeLineInImage(m_bestMatch->getPolygon(), 255, 0, 0);
//                    break;
//                }                
            }
        }
        invalidate();

        shapeSize -= 3;
    }    
    return m_bestMatch;
}

bool SvgObjectDetector::colorMatch(unsigned int failCount)
{
    Vector2<int>* ret;
    Pixel<float>* pixel;
    Line* line = m_bestMatch->getPolygon();
    unsigned int lenFourth = line->getSize() / 4;
        
    for (unsigned int i = 0; i < 2; i++)
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

void SvgObjectDetector::initDetectionParams(unsigned int shrink)
{
    DetectionParams::selectionTreshold = 30;

    DetectionParams::maxPercentageError = 0.1;
}