/* 
 * File:   Line.h
 * Author: lubos
 *
 * Created on February 10, 2013, 1:21 AM
 */

#ifndef LINE_H
#define	LINE_H


#include <vector>
#include <stdexcept>
#include <cstdlib>
#include "Vector2.h"

class Line
{
public:
    std::vector<Vector2<int> > points;    
    
    bool locked;
        
    double straightnessFactor;
    
    double direction;
    
    double directionDegrees;
    
    double length;
    
public:
    Line() 
    : locked(false), straightnessFactor(0), direction(0), directionDegrees(0), length(0) {}
    
    Line(const Line& input, unsigned int skip = 1);     
    
    Line(Line* input, unsigned int skip = 1);
    
    virtual ~Line() {}    
            
    bool isClose(Line* input, unsigned int crossCount = 0); 
    
    bool isInline(Line* input);
    
    bool isTooNarrow();    
    
    void addPoint(Vector2<int> pt);
    
    void addPoint(int x, int y);
    
    double getDistanceTo(Vector2<int>& point);
    
    void computeProperties();                
    
    void deletePoints();
    
    Vector2<int>* getPointPtr(unsigned int index);
    
    Vector2<int> getPoint(unsigned int index);
    
    void setPoint(Vector2<int> point, unsigned int index);
    
    void setPoint(int x, int y, unsigned int index);
    
    unsigned int getSize() const;
    
    friend std::ostream& operator<< (std::ostream& out, const Line& line);     
    
public:    
    static Line* getMaxLengthLine(Line** lines);    
    
    static Line* getStraightesstLine(Line** lines);   
            
    
public: //should be private
    double computeStraightnessFactor();         
    
    double computeDirectionInDegrees();
    
    double computeDirection();
    
    double computeLength();        
    
    unsigned int getCountOfPoints() const;
    
    Vector2<int> getBegin();
    
    Vector2<int> getEnd();
};


#endif	/* LINE_H */

