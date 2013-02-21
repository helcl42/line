#include "LineDetector.h"

LineDetector::LineDetector(DetectionSettings* settings) 
{
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1, &LineDetector::imageCallback, this);    
    m_resender = m_sendHandler.advertise<sensor_msgs::Image>("resender", 500);       
    m_imageService = new ImageService(settings);
    
}

LineDetector::~LineDetector() 
{
    SAFE_DELETE(m_imageService);
}

void LineDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{              
    m_imageService->perform(msg);
    m_resender.publish(msg); 
    
    //ros::shutdown();             
}

int main(int argc, char** argv)
{
    //TODO read params
    
    ros::init(argc, argv, "line");
    ROS_INFO("Line started");   
    
    DetectionSettings settings(0xdc, 0xdc, 0xdc, 120);
    LineDetector detector(&settings);
    
    ros::spin();
    ROS_INFO("Line finished");
    
    return 0;
}



