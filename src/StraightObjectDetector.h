/* 
 * File:   StraightShapeObjectDetector.h
 * Author: lubos
 *
 * Created on March 7, 2013, 3:01 AM
 */

#ifndef STRAIGHTSHAPEOBJECTDETECTOR_H
#define	STRAIGHTSHAPEOBJECTDETECTOR_H

#include "Polygon.h"
#include "AbstractDetector.h"
#include "Shapes/LinePair.h"

class StraightObjectDetector : public AbstractDetector
{
protected:    
    std::vector<Polygon<int>*> m_lines;

public:
    StraightObjectDetector(DetectionColorItem* settings = NULL);

    StraightObjectDetector(ImageMap<float>* image, Image<float>* colorImage);

    virtual ~StraightObjectDetector();
    
public:
    virtual LinePair* findObject() = 0;    
    
protected:
    Polygon<int>* getLongestLine();

    Polygon<int>* getStraightestLine();
    
    Polygon<int>* findLineWithDirection(Polygon<int>* input, float angle = 0);

    void lockAllLines(bool val);
    
    void lockSimilarLines(Polygon<int>* line, unsigned int crossCount = 0);

    void traverseImage();

    inline bool storeBestLine(Polygon<int>** lines);

    Polygon<int>* findCorrectLine(Vector2<int>* vecs, Vector2<int> pos);

    void sortLinesByStraightness(bool reverse = false);

    void sortLinesByLength(bool reverse = true);
};


#endif	/* STRAIGHTSHAPEOBJECTDETECTOR_H */

