/* 
 * File:   StraightShapeObjectDetector.h
 * Author: lubos
 *
 * Created on March 7, 2013, 3:01 AM
 */

#ifndef STRAIGHTSHAPEOBJECTDETECTOR_H
#define	STRAIGHTSHAPEOBJECTDETECTOR_H

#include "Line.h"
#include "ObjectDetector.h"

class StraightObjectDetector : public ObjectDetector
{
protected:    
    std::vector<Line*> m_lines;

public:
    StraightObjectDetector(DetectionColorItem* settings = NULL);

    StraightObjectDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~StraightObjectDetector();
    
protected:
    Line* getLongestLine();

    Line* getStraightestLine();

    void writeLineInImage(Line* line, int r, int g, int b);
    
    Line* findLineWithDirection(Line* input, float angle = 0);

    void lockAllLines(bool val);
    
    void lockSimilarLines(Line* line, unsigned int crossCount = 0);

    void traverseImage();

    inline bool storeBestLine(Line** lines);

    Line* findCorrectLine(Vector2<int>* vecs, Vector2<int> pos);

    void sortLinesByStraightness();

    void sortLinesByLength();        
};


#endif	/* STRAIGHTSHAPEOBJECTDETECTOR_H */

