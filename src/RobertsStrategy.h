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
    RobertsStrategy(DetectionLineItem* settings = NULL) 
        : AbstractStrategy(settings) {}
    
    RobertsStrategy(Image<float>* image, DetectionLineItem* settings = NULL) 
        : AbstractStrategy(image, settings) {}
                               
    virtual ~RobertsStrategy() {}        
        
    LinePair* detectLine();
    
    Rectangle* detectRectangle();
    
protected:        
    void robertsAlgorithm();            
};

#endif	/* ROBERTSSTRATEGY_H */

