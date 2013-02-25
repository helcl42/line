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
    
    DetectionLineItem() {}
    
    ~DetectionLineItem() {}        
};


struct DetectionSettings 
{
    std::vector<DetectionLineItem*> colors;
    
    PixelRGB<unsigned char>  searchedColor;   
    
    DetectionSettings() {}
    
    //format #334432 #657687 #657654 ...
    DetectionSettings(int argc, char** argv)
    {
        for(int i = 0; i < argc; i++)
        {
            std::string color(argv[i]);
            if(color[0] == '#' || color.length() == 7)
            {
                //todo parse color
                colors.push_back(new DetectionLineItem());
            }
            else 
            {
                throw std::runtime_error("Invalid color format");
            }
                
        }
    }    
    
    DetectionSettings(unsigned char r, unsigned char g, unsigned char b) 
    {
        searchedColor.r = r;
        searchedColor.g = g;
        searchedColor.b = b;        
    }        
    
    ~DetectionSettings() {}
    
    void addLineItem(unsigned char r, unsigned char g, unsigned char b) 
    {
        DetectionLineItem item;
        item.color = PixelRGB<unsigned char>(r, g, b);        
        colors.push_back(&item);
    }
    
    DetectionLineItem* operator[](int index)
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
    
    std::ostream& operator<<(std::ostream& out, const DetectionSettings& settings)
    {
        
        return out;
    }
};

#endif	/* DETECTIONSETTINGS_H */

