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
#include "Image.h"
#include "Timer.h"
#include "DetectionSettings.h"
#include "LineDetector.h"

class ImageService
{
private:                    
    unsigned int m_shrink;
    
    Timer m_shrinkTimer;
    
    Timer m_changeColorTimer;
    
    Image<float>* m_image;
    
    Image<float>* m_colorImage;
    
    DetectionSettings* m_settings;    
    
    unsigned int m_settingsIndex;
    
    LineDetector* m_lineDetector;
    
public:
    ImageService(DetectionSettings* settings);    
    
    ~ImageService();  
    
    //temp
    void test(const sensor_msgs::Image::ConstPtr& depth);
    
    void perform(const sensor_msgs::Image::ConstPtr& img, const sensor_msgs::Image::ConstPtr& depth);
    
    void writeImageToMessage(const sensor_msgs::Image::ConstPtr& img);
    
    void writeLinesToMessage(const sensor_msgs::Image::ConstPtr& img, Line** line, unsigned int count, unsigned int width = 1);        
    
private:
    Vector2<float>* getWayPoint(LinePair* line, const sensor_msgs::Image::ConstPtr& depth);
};

#endif	/* IMAGEMESSAGESERVICE_H */

