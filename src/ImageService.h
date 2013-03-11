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
#include "RectangleDetector.h"

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
    
    RectangleDetector* m_rectangleDetector;
    
public:
    ImageService(DetectionSettings* settings);    
    
    ~ImageService();      
    
    Vector2<int>* perform(const sensor_msgs::Image::ConstPtr& img);
    
    unsigned int getShrink() const;
    
    void writePointToMessage(const sensor_msgs::Image::ConstPtr& img, Vector2<int>* point, unsigned int size = 3);
    
    void writeImageToMessage(const sensor_msgs::Image::ConstPtr& img);
    
    void writeLinesToMessage(const sensor_msgs::Image::ConstPtr& img, Line** line, unsigned int count, unsigned int width = 1);        
    
private:
    Vector2<int>* getObjectPoint(StraightDetectedObject* line);
};

#endif	/* IMAGEMESSAGESERVICE_H */

