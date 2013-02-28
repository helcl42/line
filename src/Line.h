/* 
 * File:   Line.h
 * Author: lubos
 *
 * Created on February 10, 2013, 1:21 AM
 */

#ifndef LINE_H
#define	LINE_H


#include <vector>
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
    
    Line(const Line& input);     
    
    virtual ~Line() {}    
            
    bool isClose(Line* input); 
    
    bool isInline(Line* input);
    
    bool addPoint(Vector2<int> pt);
    
    double getDistanceTo(Vector2<int>& point);
    
    void computeProperties();                
    
    friend std::ostream& operator<< (std::ostream& out, const Line& line);     
    
public:    
    static Line* getMaxLengthLine(Line** lines);    
    
    static Line* getStraightesstLine(Line** lines);   
    
public: //should be private
    double computeStraightnessFactor();         
    
    double computeDirectionInDegrees();
    
    double computeDirection();
    
    double computeLength();
};


#endif	/* LINE_H */

