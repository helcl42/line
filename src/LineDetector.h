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

    LinePair* detectLine();
        
    void invalidate();
    
    LinePair* findBestLine();
    
protected:
    
    bool lineColorMatch(Line* l1, Line* l2);    

private:
    static void initDetectionParams();
};


#endif	/* LINEDETECTOR_H */

