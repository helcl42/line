/* 
 * File:   DetectionSettings.h
 * Author: lubos
 *
 * Created on January 27, 2013, 9:09 AM
 */

#ifndef DETECTIONSETTINGS_H
#define	DETECTIONSETTINGS_H

#include <vector>
#include <cstdlib>
#include "Pixel/Pixel.h"
#include "Utils/Utils.h"


class DetectionColorItem 
{
public:
    PixelRGB<float> color;
    
public:    
    DetectionColorItem() {}
    
    ~DetectionColorItem() {}        
};


class DetectionSettings 
{
public:
    std::vector<DetectionColorItem*> colors;  
    
    unsigned int colorIndex;
    
public:            
    //format 334432 657687 657654 ...
    DetectionSettings(int argc, char** argv);
    
    ~DetectionSettings();        
    
    unsigned int getCountOfColors() const;
    
    DetectionColorItem* getItem(int index) const;
    
    DetectionColorItem* getNext();
    
    DetectionColorItem* operator[](int index);
    
    friend std::ostream& operator<<(std::ostream& out, const DetectionSettings& settings);
};

#endif	/* DETECTIONSETTINGS_H */

