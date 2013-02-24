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
#include "DetectionParams.h"
#include "BestLine.h"

class AbstractStrategy 
{
protected:
    BmpImage<float>* m_workImage;
    
    BmpImage<float>* m_colorImage;
    
    DetectionSettings* m_settings;
    
    PixelRGB<float> m_baseColor;            
    
    std::vector<Line*> m_lines;
    
    BestLine* m_bestLine;
    
public:
    AbstractStrategy(DetectionSettings* settings);         
    
    AbstractStrategy(BmpImage<float>* image, DetectionSettings* settings);         
        
    virtual ~AbstractStrategy();          
    
     void smooth();
     
     void gaussianBlur();
     
     void sharpen();
     
     Line* getLongestLine();
     
     Line* getStraightestLine();
     
     void traverseImage();
     
     BestLine* findBestLine();
     
     void setImages(BmpImage<float>* image, BmpImage<float>* colorImage);
     
     void cleanUp();
          
     virtual BestLine* detectLine() = 0;        
     
protected:     
     inline bool storeBestLine(Line** lines);     
     
     //Line* findCorrectLine2(int vecY, int vecX, int chX, int chY, unsigned int posY, unsigned int posX);
     
     Line* findCorrectLine(Vector2<int>* vecs, Vector2<int>& pos);
     
     void writeLineInImage(Line* line, int r, int g, int b);
     
     Line* findLineWithSameDirection(Line* input);
     
     void resolveSimilarColor(int interval = 50);     
     
     void replaintSimilarColorPlaces(int interval = DetectionParams::colorTolerance);
     
     void lockSimilarLines(Line* line);
     
     void lockAllLines(bool val);          
     
private:     
     void setBaseColor();     
     
     void sortLinesByStraightness();
     
     void sortLinesByLength();
     
     bool lineColorMatch(Line* l1, Line* l2);
     
     void lockedCount();                    
};

#endif	/* IDETECTSTRATEGY_H */

