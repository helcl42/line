/* 
 * File:   CameraService.h
 * Author: lubos
 *
 * Created on March 10, 2013, 2:05 PM
 */

#ifndef CAMERASERVICE_H
#define	CAMERASERVICE_H

#include <sensor_msgs/Image.h>
#include "Utils.h"

class CameraService
{
public:
    CameraService() {}
    
    ~CameraService() {}
    
public:
    double getCameraYPosition(const sensor_msgs::Image::ConstPtr& msg);
    
    std::vector<float> getCameraAngles(const sensor_msgs::Image::ConstPtr& msg);

private:    
    double getCameraHeight(float distLow, float distHigh);
};

#endif	/* CAMERASERVICE_H */

