#include "LineDetector.h"
#include "DetectionSettings.h"


/**
 * 
 * Params:
 * bb4a1e - red
 * 1f9ea7 - blue
 * d78716 - orange
 * 6cbe40 - green
 * c7deb2 - white
 * 
 * @param argc
 * @param argv
 * @return 
 */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line");
    
    if(argc < 2)
    {
        std::cout << "Spatne zadane parametry: rosrun line line barva1 barva2 ..." << std::endl;       
        return 1;
    }
    
    DetectionSettings settings(argc, argv);
    //std::cout << settings << std::endl;    
    
    ROS_INFO("Line started");               
    
    LineDetector detector(&settings);
    detector.run();
    
    ros::spin();
    ROS_INFO("Line finished");

    return 0;
}

