#include "DetectorTopic.h"

DetectorTopic::DetectorTopic(std::vector<DetectedObject*>& shapes, DetectionSettings* settings)
: m_objectPoint(NULL), m_shrink(1), m_cameraY(0)
{
    m_imageService = new ImageService(shapes, settings);
}

DetectorTopic::~DetectorTopic()
{
    SAFE_DELETE(m_imageService);
}

void DetectorTopic::run()
{    
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1, &DetectorTopic::imageCallback, this);
    m_subDepth = m_handlerDepth.subscribe("/camera/depth/image", 1, &DetectorTopic::depthImageCallback, this);
    m_resender = m_sendHandler.advertise<sensor_msgs::Image > ("resender", 1);
}

void DetectorTopic::depthImageCallback(const sensor_msgs::Image::ConstPtr& depth)
{
    float distance, midDistance, ground;    
    Vector2<float> resultPoint;
    double tempCamY = m_cameraService.getCameraYPosition(depth);      
    
    if (!isnan(tempCamY) && tempCamY > 0)
    {
        m_cameraY = tempCamY;
        m_cameraGroundAngles = m_cameraService.getCameraAngles(depth);
    }    
    
    std::cout << "Cam Height!!!!: " << m_cameraY << std::endl;

    if (m_objectPoint != NULL)
    {                
        BYTES_TO_FLOAT_L(distance, depth->data, m_objectPoint->x * 4 * m_shrink + m_objectPoint->y * depth->width * 4 * m_shrink);
        BYTES_TO_FLOAT_L(midDistance, depth->data, depth->width * 2 * m_shrink + m_objectPoint->y * depth->width * 4 * m_shrink);       
        
        ground = pow(distance * distance - m_cameraY * m_cameraY, 0.5);        
        
        resultPoint.y = pow(midDistance * midDistance - m_cameraY * m_cameraY, 0.5);        
        if(ground > resultPoint.y)
        {
            resultPoint.x = pow(ground * ground - resultPoint.y * resultPoint.y, 0.5);
        }
        else 
        {
            resultPoint.x = pow(resultPoint.y * resultPoint.y - ground * ground, 0.5);
        }
        
        if(m_objectPoint->x * m_shrink > depth->width / 2)
        {
            resultPoint.x = -resultPoint.x;
        }
        
        std::cout << "X = " << resultPoint.x << " Y = " << resultPoint.y << std::endl;
        
        //m_moveService.moveTo(&resultPoint);
        
        SAFE_DELETE(m_objectPoint);
    }
}

void DetectorTopic::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    m_shrink = m_imageService->getShrink();
    m_objectPoint = m_imageService->perform(msg, m_cameraGroundAngles);
    m_resender.publish(msg);

}
