/* 
 * File:   IDetectStrategy.h
 * Author: lubos
 *
 * Created on January 27, 2013, 9:06 AM
 */

#ifndef IDETECTSTRATEGY_H
#define	IDETECTSTRATEGY_H

#include <vector>
#include <stdexcept>

#include "BmpImage.h"
#include "Line.h"
#include "DetectionSettings.h"

#define COLOR_TOLERANCE 130

#define LINE_LENGTH_TRESHOLD  150
#define COLOR_TRESHOLD 		  10

class AbstractStrategy 
{
protected:
    BmpImage<float>* m_bmpImage;
    
    DetectionSettings* m_settings;
    
    PixelRGB<float> m_baseColor;            
    
public:
    AbstractStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL) 
        : m_bmpImage(image), m_settings(settings) {}
    
     void smooth();
     
     void gaussianBlur();
     
     void sharpen();
     
     Line* getLongestLine(std::vector<Line*>& lines);
     
     Line* traverseImage();
     
     Line* findCorrectLine(int vecY, int vecX, unsigned int posY, unsigned int posX);
     
     void replaintSimilarColorPlaces(int interval = COLOR_TOLERANCE);
     
     virtual Line* detectLine() = 0;        
};

#endif	/* IDETECTSTRATEGY_H */

