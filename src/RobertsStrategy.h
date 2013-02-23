/* 
 * File:   RobertsStrategy.h
 * Author: lubos
 *
 * Created on February 10, 2013, 1:40 AM
 */

#ifndef ROBERTSSTRATEGY_H
#define	ROBERTSSTRATEGY_H

#include "AbstractStrategy.h"

class RobertsStrategy : public AbstractStrategy
{   
public:
    RobertsStrategy(DetectionSettings* settings = NULL) 
        : AbstractStrategy(settings) {}
    
    RobertsStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL) 
        : AbstractStrategy(image, settings) {}
                               
    virtual ~RobertsStrategy() {}        
        
    BestLine* detectLine();
    
protected:        
    void robertsAlgorithm();            
};

#endif	/* ROBERTSSTRATEGY_H */

