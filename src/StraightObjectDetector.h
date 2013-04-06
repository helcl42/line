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
#include "Shapes/LinePair.h"

class StraightObjectDetector : public ObjectDetector
{
protected:    
    std::vector<Line*> m_lines;

public:
    StraightObjectDetector(DetectionColorItem* settings = NULL);

    StraightObjectDetector(ImageMap<float>* image, Image<float>* colorImage);

    virtual ~StraightObjectDetector();
    
public:
    virtual LinePair* findObject() = 0;    
    
protected:
    Line* getLongestLine();

    Line* getStraightestLine();
    
    Line* findLineWithDirection(Line* input, float angle = 0);

    void lockAllLines(bool val);
    
    void lockSimilarLines(Line* line, unsigned int crossCount = 0);

    void traverseImage();

    inline bool storeBestLine(Line** lines);

    Line* findCorrectLine(Vector2<int>* vecs, Vector2<int> pos);

    void sortLinesByStraightness(bool reverse = false);

    void sortLinesByLength(bool reverse = true);
};


#endif	/* STRAIGHTSHAPEOBJECTDETECTOR_H */

