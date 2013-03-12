#include "DetectorTopic.h"
#include "DetectionSettings.h"


/**
 * 
 * Params
 * c88553 - red - 875349
 * 6cabeb - blue - 7cacc3
 * ceb66e - orange
 * 6cbe40 - green
 * c7deb2 - white
 * 
 * rosrun line line c88553 6cabeb ceb66e
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
    
    DetectionSettings* settings = new DetectionSettings(argc, argv);
    std::cout << *settings << std::endl;
    
    ROS_INFO("Line started");               
    
    DetectorTopic detector(settings);
    detector.run();
    
    ros::spin();
    ROS_INFO("Line finished");
    SAFE_DELETE(settings);

    return 0;
}

