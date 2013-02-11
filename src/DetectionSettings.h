/* 
 * File:   DetectionSettings.h
 * Author: lubos
 *
 * Created on January 27, 2013, 9:09 AM
 */

#ifndef DETECTIONSETTINGS_H
#define	DETECTIONSETTINGS_H

#include "Pixel.h"


struct DetectionSettings 
{
    PixelRGB<unsigned char>  searchedColor;
    double           lineWidth;
    
    DetectionSettings(unsigned char r, unsigned char g, unsigned char b, float width) 
    {
        searchedColor.r = r;
        searchedColor.g = g;
        searchedColor.b = b;
        lineWidth = width;
    }
    
    ~DetectionSettings() {}
};

#endif	/* DETECTIONSETTINGS_H */

