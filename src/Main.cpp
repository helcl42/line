#include "LineDetector.h"
#include "DetectionSettings.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "line");
    
    if(argc < 2)
    {
        std::cout << "Spatne zadane parametry: rosrun line line barva1 barva2 ..." << std::endl;       
        return 1;
    }
    
    DetectionSettings settings(argc, argv);
    
    ROS_INFO("Line started");               
    
    LineDetector detector(&settings);
    detector.run();
    
    ros::spin();
    ROS_INFO("Line finished");

    return 0;
}

