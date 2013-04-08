/* 
 * File:   SvgObjectDetectorThread.h
 * Author: lubos
 *
 * Created on April 8, 2013, 7:56 PM
 */

#ifndef OBJECTDETECTORTHREAD_H
#define	OBJECTDETECTORTHREAD_H

#include "Threading/Thread.h"
#include "Shapes/DetectedObject.h"
#include "Image/ImageMap.h"

class ObjectDetectorThread : public Thread
{
private:
    std::vector<DetectedObject*> m_objects;

    bool m_found;

    DetectedObject* m_foundObject;

    ImageMap<float>* m_workImage;

public:

    ObjectDetectorThread();

    ObjectDetectorThread(std::vector<DetectedObject*>& objects);

    ~ObjectDetectorThread();

public:
    void setInstance(ImageMap<float>* image, std::vector<DetectedObject*> objects);

    void ThreadProcedure();

    void cleanUp();

    bool found() const;

    DetectedObject* getFoundObject() const;

protected:
    bool findShapeInImage(DetectedObject* object);

    bool innerShapeFind(DetectedObject* object, unsigned int y, unsigned int x);

    bool rawShapeFind(DetectedObject* object, unsigned int y, unsigned int x);
};

#endif	/* OBJECTDETECTORTHREAD_H */

