/* 
 * File:   KirshStrategy.h
 * Author: lubos
 *
 * Created on February 10, 2013, 3:26 AM
 */

#ifndef KIRSHSTRATEGY_H
#define	KIRSHSTRATEGY_H

#include "AbstractStrategy.h"

class KirshStrategy : public AbstractStrategy
{   
public:
    KirshStrategy(DetectionSettings* settings = NULL) 
        : AbstractStrategy(settings) {}
    
    KirshStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL) 
        : AbstractStrategy(image, settings) {}
                               
    BestLine* detectLine();
    
protected:    
    void kirshAlgorithm();        
};

#endif	/* KIRSHSTRATEGY_H */

