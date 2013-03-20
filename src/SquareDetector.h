/* 
 * File:   SquareDetector.h
 * Author: lubos
 *
 * Created on March 18, 2013, 4:00 AM
 */

#ifndef SQUAREDETECTOR_H
#define	SQUAREDETECTOR_H

#include "Square.h"
#include "ObjectDetector.h"


class SquareDetector : public ObjectDetector
{
protected:            
    std::vector<float> m_angles;
    
    std::vector<Square*> m_squares;
    
    Square* m_bestSquare;
    
    Line* m_tempLine;

public:
    SquareDetector(DetectionColorItem* settings = NULL);

    SquareDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~SquareDetector();

public:
    Square* findObject();

    void invalidate();

    void initDetectionParams(unsigned int shrink = 1);
        
    void setAngles(std::vector<float> angles);

protected:
    Square* findBestSquare();

    bool colorMatch(unsigned int failCount = 0);
    
    Square* generateSquare(int x0, int y0, int size, float angle);
    
    void generateSquares(unsigned int ellipseSize);        
    
    bool findSquareInImagePart(unsigned int imagePart, unsigned int ellipseSize);
    
    bool innerSquareFind(Square* line, unsigned int y, unsigned int x);    
    
    bool rawSquareFind(Square* square, unsigned int y, unsigned int x);
};

#endif	/* SQUAREDETECTOR_H */

