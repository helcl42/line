/* 
 * File:   SobelStrategy.h
 * Author: lubos
 *
 * Created on January 27, 2013, 10:14 AM
 */

#ifndef SOBELSTRATEGY_H
#define	SOBELSTRATEGY_H

#include "AbstractStrategy.h"


class SobelStrategy : public AbstractStrategy
{   
public:
    SobelStrategy(DetectionLineItem* settings = NULL) 
        : AbstractStrategy(settings) {}
    
    SobelStrategy(BmpImage<float>* image, DetectionLineItem* settings = NULL) 
        : AbstractStrategy(image, settings) {}
        
    virtual ~SobelStrategy() {}
        
    LinePair* detectLine();
    
    Rectangle* detectRectangle();    
    
protected:    
    void sobelAlgorithm();        
};

#endif	/* SOBELSTRATEGY_H */

