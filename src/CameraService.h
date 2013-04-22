/* 
 * File:   CameraService.h
 * Author: lubos
 *
 * Created on March 10, 2013, 2:05 PM
 */

#ifndef CAMERASERVICE_H
#define	CAMERASERVICE_H

#include <sensor_msgs/Image.h>
#include "Utils/Utils.h"

class CameraService
{
private:
    static CameraService* m_instance;        
    
public:    
    static CameraService* getInstance();
    
    ~CameraService() {}
    
public:        
    double getCameraYPosition(const sensor_msgs::Image::ConstPtr& msg);
    
    std::vector<float> getCameraAngles(const sensor_msgs::Image::ConstPtr& msg);

private:    
    CameraService() {}
    
    double getCameraHeight(float distLow, float distHigh);
};

#endif	/* CAMERASERVICE_H */

