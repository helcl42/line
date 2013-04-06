/* 
 * File:   LineDetector.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:41 AM
 */

#ifndef LINEDETECTOR_H
#define	LINEDETECTOR_H

#include "Shapes/LinePair.h"
#include "StraightObjectDetector.h"

class LineDetector : public StraightObjectDetector
{
private:
    LinePair* m_bestLine;

public:
    LineDetector(DetectionColorItem* settings = NULL);

    LineDetector(ImageMap<float>* image, Image<float>* colorImage);

    virtual ~LineDetector();

public:    
    void invalidate();    
    
    LinePair* findObject();
    
    void initDetectionParams(unsigned int shrink = 1);
    
protected:            
    LinePair* findBestLine();   
    
    bool colorMatch(unsigned int failCount = 0);
};


#endif	/* LINEDETECTOR_H */

