#include "LineDetector.h"
#include "DetectionSettings.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "line");
    ROS_INFO("Line started");   
        
    DetectionSettings settings(argc, argv);
    
    LineDetector detector(&settings);
    detector.run();
    
    ros::spin();
    ROS_INFO("Line finished");

    return 0;
}

