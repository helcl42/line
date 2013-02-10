#include "LineDetector.h"

ImageWorker::ImageWorker()
{
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1, &ImageWorker::imageCallback, this);    
    m_resender = m_sendHandler.advertise<sensor_msgs::Image>("resender", 1);   
}

ImageWorker::~ImageWorker()
{    
}

void ImageWorker::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{                    
    Timer t1;    
    t1.start();    
    m_image = new BmpImage<float>(msg);
        
    DetectionSettings settings(0xdc, 0xdc, 0xdc, 120);        
    SobelStrategy sobelStrategy(m_image, &settings);    
    Line* line = sobelStrategy.detectLine();                    
    
    t1.stop();
    t1.logTime();
    
    m_image->writeToMessage(msg);
    
    SAFE_DELETE(line);
    SAFE_DELETE(m_image);
        
    m_resender.publish(msg);        
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line");
    ROS_INFO("Line started");   
    
    ImageWorker worker;
    
    ros::spin();
    ROS_INFO("Line finished");
    
    return 0;
}



