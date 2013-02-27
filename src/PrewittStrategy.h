/* 
 * File:   PrewittStrategy.h
 * Author: lubos
 *
 * Created on February 10, 2013, 5:09 PM
 */

#ifndef PREWITTSTRATEGY_H
#define	PREWITTSTRATEGY_H

#include "AbstractStrategy.h"

class PrewittStrategy : public AbstractStrategy
{   
public:
    PrewittStrategy(DetectionLineItem* settings = NULL) 
        : AbstractStrategy(settings) {}
    
    PrewittStrategy(BmpImage<float>* image, DetectionLineItem* settings = NULL) 
        : AbstractStrategy(image, settings) {}
                           
    virtual ~PrewittStrategy() {}        
        
    LinePair* detectLine();
    
protected:        
    void prewittAlgorithm();            
};

#endif	/* PREWITTSTRATEGY_H */

