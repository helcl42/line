/* 
 * File:   LineDetector.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:41 AM
 */

#ifndef LINEDETECTOR_H
#define	LINEDETECTOR_H

#include "LinePair.h"
#include "StraightObjectDetector.h"

class LineDetector : public StraightObjectDetector
{
private:
    LinePair* m_bestLine;

public:
    LineDetector(DetectionColorItem* settings = NULL);

    LineDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~LineDetector();

public:    
    void invalidate();    
    
    LineDescribableObject* findObject();
    
    void initDetectionParams(unsigned int shrink = 1);
    
protected:            
    LineDescribableObject* findBestLine();   
    
    bool colorMatch(unsigned int failCount = 0);
};


#endif	/* LINEDETECTOR_H */

