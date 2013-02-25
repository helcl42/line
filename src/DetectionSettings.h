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
#include "Pixel.h"


class DetectionLineItem 
{
public:
    PixelRGB<int> color;
    
public:    
    DetectionLineItem() {}
    
    ~DetectionLineItem() {}        
};


class DetectionSettings 
{
public:
    std::vector<DetectionLineItem*> colors;
    
    PixelRGB<unsigned char>  searchedColor;   
    
public:            
    //format 334432 657687 657654 ...
    DetectionSettings(int argc, char** argv);
    
    ~DetectionSettings();        
    
    unsigned int getCountOfColors() const;
    
    DetectionLineItem* getItem(int index) const;
    
    DetectionLineItem* operator[](int index);
    
    friend std::ostream& operator<<(std::ostream& out, const DetectionSettings& settings);
};

#endif	/* DETECTIONSETTINGS_H */

