#include "DetectorTopic.h"
#include "DetectionSettings.h"
#include "SvgExtractor/SvgExtractor.h"

/**
 * 
 * Params
 * c88553 - red - 875349
 * 6cabeb - blue - 7cacc3 - 97d5fe
 * ceb66e - orange - e9cd8f
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

    if (argc < 3)
    {
        std::cout << "Spatne zadane parametry: rosrun line line shapeFile.svg lineColor1 lineColor2 ..." << std::endl;
        return 1;
    }

    SvgExtractor svgExtractor(argv[1]);

    std::vector<DetectedObject*> shapes = svgExtractor.extractSvgObjects();

    DetectionSettings* settings = new DetectionSettings(argc, argv);
    std::cout << *settings << std::endl;

    ROS_INFO("Line started");

    DetectorTopic detector(shapes, settings);
    detector.run();

    ros::spin();
    ROS_INFO("Line finished");
    SAFE_DELETE(settings);

    return 0;
}

