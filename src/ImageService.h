/* 
 * File:   ImageMessageService.h
 * Author: lubos
 *
 * Created on February 21, 2013, 11:09 AM
 */

#ifndef IMAGEMESSAGESERVICE_H
#define	IMAGEMESSAGESERVICE_H

#include <sensor_msgs/Image.h>

#include "Line.h"
#include "BmpImage.h"
#include "Timer.h"
#include "DetectionSettings.h"
#include "SobelStrategy.h"

class ImageService
{
private:                    
    unsigned int m_shrink;
    
    Timer m_shrinkTimer;
    
    Timer m_changeColorTimer;
    
    BmpImage<float>* m_image;
    
    BmpImage<float>* m_colorImage;
    
    DetectionSettings* m_settings;    
    
    unsigned int m_settingsIndex;
    
    AbstractStrategy* m_strategy;        
    
public:
    ImageService(DetectionSettings* settings);    
    
    ~ImageService();  
    
    //void setInstance(const sensor_msgs::Image::ConstPtr& img);
    
    void perform(const sensor_msgs::Image::ConstPtr& img);
    
    void writeImageToMessage(const sensor_msgs::Image::ConstPtr& img);
    
    void writeLineToMessage(const sensor_msgs::Image::ConstPtr& img, Line** line, unsigned int width);        
};

#endif	/* IMAGEMESSAGESERVICE_H */

