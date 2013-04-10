/* 
 * File:   LineDetector.h
 * Author: lubos
 *
 * Created on February 9, 2013, 8:38 PM
 */

#ifndef DETECTORTOPIC_H
#define	DETECTORTOPIC_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cstdio>
#include <ctime>

#include "Utils/Utils.h"
#include "Utils/Timer.h"
#include "DetectionSettings.h"
#include "Image/Image.h"
#include "ImageService.h"
#include "CameraService.h"
#include "MoveService.h"

class DetectorTopic
{
private:            
    Vector2<int>* m_objectPoint;
    
    unsigned int m_shrink;
        
    double m_cameraY;
    
    std::vector<float> m_cameraGroundAngles;
    
    ros::NodeHandle m_handler;        
    
    ros::NodeHandle m_handlerDepth;
    
    ros::Subscriber m_sub;
    
    ros::Subscriber m_subDepth;         
    
    ros::NodeHandle m_sendHandler;        
    
    ros::Publisher m_resender;            
    
    ImageService* m_imageService;                
    
    CameraService m_cameraService;
    
    MoveService m_moveService;
            
public:
    DetectorTopic(std::vector<DetectedObject*>& shapes, DetectionSettings* settings);
    
    ~DetectorTopic();
    
    void run();        

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg_ptr);       
    
    void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg);        
};


#endif	/* DETECTORTOPIC_H */

