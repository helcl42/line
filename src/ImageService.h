/* 
 * File:   ImageMessageService.h
 * Author: lubos
 *
 * Created on February 21, 2013, 11:09 AM
 */

#ifndef IMAGEMESSAGESERVICE_H
#define	IMAGEMESSAGESERVICE_H

#include <vector>
#include <sensor_msgs/Image.h>

#include "Line.h"
#include "Image/Image.h"
#include "Utils/Timer.h"
#include "DetectionSettings.h"
#include "SvgObjectDetector.h"
#include "LineDetector.h"
#include "Image/ImageMap.h"

class ImageService
{
private:                    
    unsigned int m_shrink;
    
    Timer m_shrinkTimer;
    
    Timer m_changeColorTimer;
    
    ImageMap<float>* m_image;
    
    Image<float>* m_colorImage;
    
    DetectionSettings* m_settings;    
    
    unsigned int m_settingsIndex;
        
    bool m_lookUpLines;
        
    LineDetector* m_lineDetector;          
    
    SvgObjectDetector* m_objectDetector;
    
public:
    ImageService(std::vector<DetectedObject*>& shapes, DetectionSettings* settings);    
    
    ~ImageService();      
    
    Vector2<int>* perform(const sensor_msgs::Image::ConstPtr& img, std::vector<float> cameraGroundAngles);
    
    unsigned int getShrink() const;
    
    void writePointToMessage(const sensor_msgs::Image::ConstPtr& img, Vector2<int>* point, unsigned int size = 3);
    
    void writeImageMapToMessage(const sensor_msgs::Image::ConstPtr& img);        
    
    void writeImageToMessage(const sensor_msgs::Image::ConstPtr& img);        
    
    void writeLinesToMessage(const sensor_msgs::Image::ConstPtr& img, Line** line, unsigned int count, unsigned int width = 1);        
    
private:        
    void tryChangeSettings();        
};

#endif	/* IMAGEMESSAGESERVICE_H */

