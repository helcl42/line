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
    PrewittStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL) 
        : AbstractStrategy(image, settings) {}
                           
    virtual ~PrewittStrategy() {}        
        
    Line** detectLine();
    
protected:        
    void prewittAlgorithm();            
};

#endif	/* PREWITTSTRATEGY_H */

