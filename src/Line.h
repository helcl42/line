/* 
 * File:   Line.h
 * Author: lubos
 *
 * Created on February 10, 2013, 1:21 AM
 */

#ifndef LINE_H
#define	LINE_H

#include <sensor_msgs/Image.h>
#include "Vector2.h"

struct Line
{
    std::vector<Vector2<int> > points;

    Line() {}

    Line(Line* input);
    
    ~Line() {}

    
    bool isSimilar(Line* input); 

    void writeToMessage(const sensor_msgs::Image::ConstPtr& img);                    
    
    friend std::ostream& operator<< (std::ostream& out, const Line& line);     
    
    
    static unsigned int getMaxLengthIndex(Line** lines);    
    
    static Vector2<int> getDirection(Line* line1, Line* line2);
};

#endif	/* LINE_H */

