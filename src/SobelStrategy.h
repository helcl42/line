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
    SobelStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL) 
        : AbstractStrategy(image, settings) {}
                               
    Line* detectLine();
    
protected:    
    void sobelAlgorithm();        
};

#endif	/* SOBELSTRATEGY_H */

