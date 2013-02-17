/* 
 * File:   Line.h
 * Author: lubos
 *
 * Created on February 10, 2013, 1:21 AM
 */

#ifndef LINE_H
#define	LINE_H

#include "Vector2.h"
#include <sensor_msgs/Image.h>

class Line
{
public:
    std::vector<Vector2<int> > points;
    
    float straightnessFactor;
    
public:
    Line()
    : straightnessFactor(0) {}
    
    Line(Line* input);     
    
    virtual ~Line() {}    
            
    bool isSimilar(Line* input); 
    
    void computeStraightnessFactor();         
    
    double getDirection();
    
    void writeToMessage(const sensor_msgs::Image::ConstPtr& img);                    
    
    friend std::ostream& operator<< (std::ostream& out, const Line& line);     
    
    static unsigned int getMaxLengthIndex(Line** lines);    
    
    static unsigned int getStraightestIndex(Line** lines);        
};


#endif	/* LINE_H */

