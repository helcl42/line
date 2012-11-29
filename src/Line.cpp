#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <cstdio>
#include <ctime>

#include "Utils.h"
#include "RawImage.h"

#define COLOR_TOLERANCE 50

class ImageWorker
{
private:
    ros::NodeHandle m_handler;
    ros::Subscriber m_sub;
    RawImage* m_currentImage;    
    
public:
    ImageWorker();
    ~ImageWorker();

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg_ptr);
    double diffclock(clock_t clock1, clock_t clock2);    
};


double ImageWorker::diffclock(clock_t clock1, clock_t clock2)
{
    double diffticks = clock1 - clock2;
    double diffms = (diffticks * 1000) / CLOCKS_PER_SEC;
    return diffms;
}


ImageWorker::ImageWorker() : m_currentImage(NULL)
{
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1000, &ImageWorker::imageCallback, this);
}


ImageWorker::~ImageWorker()
{
    SAFE_DELETE(m_currentImage);
}


void ImageWorker::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{                   
    clock_t begin = clock();
    
    RawImage image(msg);
    //RawImage image;    
    //image.loadBmpFile("saved_test_test_rot_3.bmp");
    
    //9a374d
    image.findPlacesWithTheSimilarColor(0x9a, 0x37, 0x4d, COLOR_TOLERANCE);
    image.blur();
    image.sobel();        
    Line* line = image.detectLines();             
    LineVector* direction = image.getDirectionOfLine(line);        
    
    image.writeBmpFile("saved_test_test_out.bmp");        
    clock_t end = clock();
    std::cout << "time elapsed is " << double(diffclock(end, begin)) << "ms" << std::endl;

    SAFE_DELETE(direction);    
    SAFE_DELETE(line);        
    
    ROS_INFO("SAVED IMAGE height [%u] width [%u] %s\n", msg->height, msg->width, msg->encoding.c_str());

    //m_sub.shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line");
    ROS_INFO("Line started");

    //ros::NodeHandle handler;
    //ros::NodeHandle handler2;
    //ros::Subscriber rgbSub = handler.subscribe("/camera/rgb/image_color", 1024, imageCallback);
    //ros::Subscriber depthSub = handler2.subscribe("/camera/depth/image_raw", 1024, depthCallback);

    ImageWorker worker;
    
    ros::spin();
    
    return 0;
}



