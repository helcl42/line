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

bool ObjectDetectorThread::rawShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    unsigned int ratio = 14, failCount = 0;
    double percentFail;
    Line* squareLine = shape->getPolygon();
    unsigned int lineSize = squareLine->getSize() / ratio;

    for (unsigned int k = 0; k < lineSize; k += ratio)
    {
        Vector2<int> point = squareLine->points[k];
        if (m_workImage->getValueAt(point.y + y, point.x + x) < DetectionParams::selectionTreshold) failCount++;
    }

    percentFail = (double) failCount / (double) lineSize;

    if (percentFail < 0.05) return true;
    return false;
}

bool ObjectDetectorThread::innerShapeFind(DetectedObject* shape, unsigned int y, unsigned int x)
{
    Line* shapeLine = shape->getPolygon();
    unsigned int lineSize = shapeLine->getSize();
    unsigned int failCount = 0;
    double percentFail;
    Vector2<int>* point;

    if (!rawShapeFind(shape, y, x)) return false;

    for (unsigned int k = 0; k < lineSize; k++)
    {
        point = shapeLine->getPointPtr(k);
        if (m_workImage->getValueAt(point->y + y, point->x + x) < DetectionParams::selectionTreshold) failCount++;
    }

    percentFail = (double) failCount / (double) lineSize;

    if (percentFail < DetectionParams::maxPercentageError)
    {
        std::cout << "failCount = " << failCount << " size = " << lineSize << " fail = " << percentFail << "%" << std::endl;
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

    for (unsigned int i = 0; i < baseHeight - offsetY - 1; i += 3)
    {
        for (unsigned int j = 0; j < baseWidth - offsetX - 1; j += 3)
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

void ObjectDetectorThread::ThreadProcedure()
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

