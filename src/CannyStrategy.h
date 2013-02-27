/* 
 * File:   CannyStrategy.h
 * Author: lubos
 *
 * Created on February 17, 2013, 2:24 AM
 */

#ifndef CANNYSTRATEGY_H
#define	CANNYSTRATEGY_H

#include "AbstractStrategy.h"
#include "BmpFileIO.h"


class CannyStrategy : public AbstractStrategy
{   
public:
    CannyStrategy(DetectionLineItem* settings = NULL) 
        : AbstractStrategy(settings) {}
    
    CannyStrategy(BmpImage<float>* image, DetectionLineItem* settings = NULL) 
        : AbstractStrategy(image, settings) {}
        
    virtual ~CannyStrategy() {}        
        
    LinePair* detectLine();
    
protected:    
    void cannyAlgorithm();        
};

#endif	/* CANNYSTRATEGY_H */

