/* 
 * File:   StraightShapeObjectDetector.h
 * Author: lubos
 *
 * Created on March 7, 2013, 3:01 AM
 */

#ifndef STRAIGHTSHAPEOBJECTDETECTOR_H
#define	STRAIGHTSHAPEOBJECTDETECTOR_H

#include "Line.h"
#include "AbstractDetector.h"
#include "Shapes/LinePair.h"

class StraightObjectDetector : public AbstractDetector
{
protected:    
    std::vector<Line<int>*> m_lines;

public:
    StraightObjectDetector(DetectionColorItem* settings = NULL);

    StraightObjectDetector(ImageMap<float>* image, Image<float>* colorImage);

    virtual ~StraightObjectDetector();
    
public:
    virtual LinePair* findObject() = 0;    
    
protected:
    Line<int>* getLongestLine();

    Line<int>* getStraightestLine();
    
    Line<int>* findLineWithDirection(Line<int>* input, float angle = 0);

    void lockAllLines(bool val);
    
    void lockSimilarLines(Line<int>* line, unsigned int crossCount = 0);

    void traverseImage();

    inline bool storeBestLine(Line<int>** lines);

    Line<int>* findCorrectLine(Vector2<int>* vecs, Vector2<int> pos);

    void sortLinesByStraightness(bool reverse = false);

    void sortLinesByLength(bool reverse = true);
};


#endif	/* STRAIGHTSHAPEOBJECTDETECTOR_H */

