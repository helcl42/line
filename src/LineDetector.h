/* 
 * File:   LineDetector.h
 * Author: lubos
 *
 * Created on February 9, 2013, 8:38 PM
 */

#ifndef LINEDETECTOR_H
#define	LINEDETECTOR_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <cstdio>
#include <ctime>

#include "Utils.h"
#include "Timer.h"
#include "DetectionSettings.h"
#include "BmpImage.h"
#include "SobelStrategy.h"

#define COLOR_TOLERANCE 50

class ImageWorker
{
private:
    ros::NodeHandle m_handler;
    ros::NodeHandle m_sendHandler;
    
    ros::Subscriber m_sub;
    ros::Publisher m_resender;        

    BmpImage<float>* m_image;
    
public:
    ImageWorker();
    ~ImageWorker();

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg_ptr);       
};


#endif	/* LINEDETECTOR_H */
