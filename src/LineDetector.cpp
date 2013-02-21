#include "LineDetector.h"

LineDetector::LineDetector(DetectionSettings* settings) 
: m_settings(settings)
{
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1, &LineDetector::imageCallback, this);    
    m_resender = m_sendHandler.advertise<sensor_msgs::Image>("resender", 500);   
    m_image = new BmpImage<float>();
}

LineDetector::~LineDetector() 
{
    SAFE_DELETE(m_image);
}

void LineDetector::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{                    
    Timer t1;    
    t1.start();    
    m_image->setInstance(msg, 4);
                   
    SobelStrategy sobelStrategy(m_image, m_settings);    
    Line** line = sobelStrategy.detectLine();                    
    
//    RobertsStrategy robertsStrategy(m_image, m_settings);    
//    Line* line = robertsStrategy.detectLine();                    
    
//    PrewittStrategy prewittStrategy(m_image, m_settings);    
//    Line* line = prewittStrategy.detectLine();  
    
    t1.stop();
    t1.logTime();
    
    m_image->writeToMessage(msg);
    
//    if(line != NULL) 
//    {
//        line->writeToMessage(msg);
//    }
    
    SAFE_DELETE(line);                
    
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



