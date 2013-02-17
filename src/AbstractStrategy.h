/* 
 * File:   IDetectStrategy.h
 * Author: lubos
 *
 * Created on January 27, 2013, 9:06 AM
 */

#ifndef IDETECTSTRATEGY_H
#define	IDETECTSTRATEGY_H

#include <vector>

#include "Line.h"
#include "BmpImage.h"
#include "DetectionSettings.h"

#define COLOR_TOLERANCE 50

#define LINE_LENGTH_TRESHOLD  180
#define COLOR_TRESHOLD 		  85
#define SELECTION_TRESHOLD    85


class AbstractStrategy 
{
protected:
    BmpImage<float>* m_bmpImage;
    
    DetectionSettings* m_settings;
    
    PixelRGB<float> m_baseColor;            
    
    std::vector<Line*> m_lines;
    
public:
    AbstractStrategy(BmpImage<float>* image, DetectionSettings* settings = NULL);         
        
    virtual ~AbstractStrategy();          
    
     void smooth();
     
     void gaussianBlur();
     
     void sharpen();
     
     Line* getLongestLine();
     
     Line* getStraightestLine();
     
     Line* traverseImage();
     
     virtual Line* detectLine() = 0;        
     
protected:     
     inline bool storeBestLine(Line** lines);     
     
     Line* findCorrectLine(int vecY, int vecX, int chX, int chY, unsigned int posY, unsigned int posX);
     
     void writeLineInImage(Line* line, int r, int g, int b);
     
     Line* findLineWithBestPrice(Line* input);
     
     void resolveSimilarColor(int interval = 50);     
     
     void replaintSimilarColorPlaces(int interval = COLOR_TOLERANCE);
     
     void removeSimilarLines(Line* line);
     
private:     
     void setBaseColor();
          
};

#endif	/* IDETECTSTRATEGY_H */

