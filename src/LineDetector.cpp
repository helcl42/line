#include "LineDetector.h"

LineDetector::LineDetector(DetectionSettings* settings)
{    
    m_imageService = new ImageService(settings);        
}

LineDetector::~LineDetector()
{
    SAFE_DELETE(m_imageService);
}

void LineDetector::run()
{
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1, &LineDetector::imageCallback, this);
    m_resender = m_sendHandler.advertise<sensor_msgs::Image > ("resender", 500);
}

void LineDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    m_imageService->perform(msg);
    m_resender.publish(msg);   
}



