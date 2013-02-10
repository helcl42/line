/* 
 * File:   IDetectStrategy.h
 * Author: lubos
 *
 * Created on January 27, 2013, 9:06 AM
 */

#ifndef IDETECTSTRATEGY_H
#define	IDETECTSTRATEGY_H

#include <vector>

#include "BmpImage.h"
#include "Vector2.h"
#include "DetectionSettings.h"

#define COLOR_TOLERANCE 50

class AbstractStrategy 
{
protected:
    BmpImage<float>* m_bmpImage;
    
    DetectionSettings* m_settings;
    
    PixelRGB<float> m_baseColor;            
    
public:
    AbstractStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL) 
        : m_bmpImage(image), m_settings(settings) {}
    
     void blur();
     
     void replaintSimilarColorPlaces(int interval = COLOR_TOLERANCE);
     
     virtual Line* detectLine() = 0;    
};

#endif	/* IDETECTSTRATEGY_H */

