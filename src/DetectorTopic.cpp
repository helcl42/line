#include "DetectorTopic.h"


DetectorTopic::DetectorTopic(DetectionSettings* settings)
: m_objectPoint(NULL), m_shrink(0)
{
    m_imageService = new ImageService(settings);
}

DetectorTopic::~DetectorTopic()
{
    SAFE_DELETE(m_imageService);
}

void DetectorTopic::run()
{
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1, &DetectorTopic::imageCallback, this);
    m_subDepth = m_handlerDepth.subscribe("/camera/depth/image", 1, &DetectorTopic::depthImageCallback, this);
    m_resender = m_sendHandler.advertise<sensor_msgs::Image > ("resender", 500);
}

void DetectorTopic::depthImageCallback(const sensor_msgs::Image::ConstPtr& depth)
{
    if (m_objectPoint != NULL)
    {
        float distance;
        unsigned int index;

        index = m_objectPoint->x * 4 * m_shrink + m_objectPoint->y * depth->width * 4 * m_shrink;
        BYTES_TO_FLOAT_L(distance, depth->data, index);
        std::cout << "distance = " << distance << std::endl;

        //send waypoint

        SAFE_DELETE(m_objectPoint);       
    }
    
    float cameraY;
    
    if(!m_checkCameraY.isStarted()) 
    {
        m_checkCameraY.start();
    }
    else if(m_checkCameraY.isElapsedMs(8000)) 
    {
        
        cameraY = getCameraYPosition(depth);
        std::cout << cameraY << std::endl;
    }
}

void DetectorTopic::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    m_shrink = m_imageService->getShrink();
    m_objectPoint = m_imageService->perform(msg);
    m_resender.publish(msg);

}


float DetectorTopic::getCameraYPosition(const sensor_msgs::Image::ConstPtr& msg)
{
    float highMidDistance;
    float lowMidDistance;
    BYTES_TO_FLOAT_L(highMidDistance, msg->data, 4 * msg->width / 2);
    BYTES_TO_FLOAT_L(lowMidDistance, msg->data, (4 * msg->width * msg->height) - (4 * msg->width / 2));
    
    std::cout << "High = " << highMidDistance << " Low = " << lowMidDistance << std::endl;
    //fake
    return highMidDistance;
}


