/* 
 * File:   SobelStrategy.h
 * Author: lubos
 *
 * Created on January 27, 2013, 10:14 AM
 */

#ifndef SOBELSTRATEGY_H
#define	SOBELSTRATEGY_H

#include "AbstractStrategy.h"

#define LINE_LENGTH_TRESHOLD  150
#define COLOR_TRESHOLD 		  100


class SobelStrategy : public AbstractStrategy
{   
public:
    SobelStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL) 
        : AbstractStrategy(image, settings) {}
                               
    Line* detectLine();
    
protected:
    Line* findCorrectLine(int vecY, int vecX, unsigned int posY, unsigned int posX);
    
    Line* getLongestLine(std::vector<Line*>& lines);                
    
    void sobel();    
    
    Line* traverseImage();
};

#endif	/* SOBELSTRATEGY_H */

