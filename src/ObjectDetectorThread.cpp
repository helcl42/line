#include "ObjectDetectorThread.h"
#include "Shapes/GeneralObject.h"

ObjectDetectorThread::ObjectDetectorThread()
: m_found(false), m_workImage(NULL)
{
    m_foundObject = new GeneralObject();
}

ObjectDetectorThread::ObjectDetectorThread(std::vector<DetectedObject*>& objects)
: m_objects(objects), m_found(false), m_workImage(NULL)
{
    m_foundObject = new GeneralObject();
}

ObjectDetectorThread::~ObjectDetectorThread()
{
}

void ObjectDetectorThread::cleanUp()
{
    m_found = false;
    m_objects.clear();
    m_foundObject->invalidate();
}

inline bool ObjectDetectorThread::rawShapeFind(DetectedObject* shape, unsigned int y, unsigned int x, unsigned int ratio, unsigned int base)
{
    double percentFail;
    Polygon<int>* squareLine = shape->getPolygon();
    unsigned int lineSize = squareLine->getSize();
    unsigned int failCount = 0;    

    for (unsigned int k = base; k < lineSize; k += ratio)
    {
        Vector2<int> point = squareLine->points[k];
        if (m_workImage->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold) failCount++;
    }

    percentFail = (double) failCount / ((double) lineSize / (double)ratio);

    if (percentFail < DetectionParams::maxPercentageError) return true;
    return false;
}

bool ObjectDetectorThread::innerShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    Polygon<int>* shapeLine = shape->getPolygon();
    unsigned int lineSize = shapeLine->getSize();    
    Vector2<int>* point;    
    unsigned int i = 64, j = 64, iteration = 0;

    while (rawShapeFind(shape, y, x, i, j))
    {       
        if (iteration != 0) i >>= 1;
        j = i >> 1;

        if(i == 0) break;
        
        iteration++;
    }

    if (i <= 1)
    {        
        m_foundObject->cleanUp();
        for (unsigned int k = 0; k < lineSize; k++)
        {
            point = shapeLine->getPointPtr(k);
            m_foundObject->addPoint(point->x + x, point->y + y);
        }
        m_found = true;
        return true;
    }
    return false;
}

bool ObjectDetectorThread::findShapeInImage(DetectedObject* shape)
{
    unsigned int baseHeight = m_workImage->getHeight();
    unsigned int baseWidth = m_workImage->getWidth();
    unsigned int offsetX = shape->getWidth();
    unsigned int offsetY = shape->getHeight();

    for (unsigned int i = 0; i < baseHeight - offsetY - 1; i += 4)
    {
        for (unsigned int j = 0; j < baseWidth - offsetX - 1; j += 4)
        {
            if (innerShapeFind(shape, i, j))
            {
                return true;
            }
        }
    }

    return false;
}

void ObjectDetectorThread::setInstance(ImageMap<float>* image, std::vector<DetectedObject*> objects)
{
    cleanUp();

    m_workImage = image;
    m_objects = objects;
}

void ObjectDetectorThread::threadProcedure()
{        
    for (unsigned int i = 0; i < m_objects.size(); i++)
    {
        if (findShapeInImage(m_objects[i]))
        {
            std::cout << "GOT IT!!!!" << std::endl;
            break;
        }
    }
}

bool ObjectDetectorThread::found() const
{
    return m_found;
}

DetectedObject* ObjectDetectorThread::getFoundObject() const
{
    return m_foundObject;
}

