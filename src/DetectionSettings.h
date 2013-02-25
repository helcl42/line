/* 
 * File:   DetectionSettings.h
 * Author: lubos
 *
 * Created on January 27, 2013, 9:09 AM
 */

#ifndef DETECTIONSETTINGS_H
#define	DETECTIONSETTINGS_H

#include "Pixel.h"


struct DetectionLineItem 
{
    PixelRGB<unsigned char> color;
    
    float width;
    
    DetectionLineItem() {}
    
    ~DetectionLineItem() {}        
};


struct DetectionSettings 
{
    std::vector<DetectionLineItem> colors;
    
    PixelRGB<unsigned char>  searchedColor;   
    
    DetectionSettings() {}
    
    DetectionSettings(unsigned char r, unsigned char g, unsigned char b, int w) 
    {
        searchedColor.r = r;
        searchedColor.g = g;
        searchedColor.b = b;        
    }        
    
    ~DetectionSettings() {}
    
    void addLineItem(unsigned char r, unsigned char g, unsigned char b, float width) 
    {
        DetectionLineItem item;
        item.color = PixelRGB<unsigned char>(r, g, b);
        item.width = width;
        colors.push_back(item);
    }
    
    DetectionLineItem& operator[](int index)
    {
        if(index > 0 || index < colors.size())
        {
            return colors[index];
        }
        else 
        {
            throw std::runtime_error("DetectionSettings:operator[]:Invalid index");
        }            
    }
};

#endif	/* DETECTIONSETTINGS_H */

