#include "DetectorTopic.h"

DetectorTopic::DetectorTopic(DetectionSettings* settings)
: m_depthMsg(NULL)
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
    //m_subDepth = m_handlerDepth.subscribe("/camera/depth/image_rect", 1, &DetectorTopic::depthImageCallback, this);
    m_resender = m_sendHandler.advertise<sensor_msgs::Image > ("resender", 500);
}

void DetectorTopic::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    m_depthMsg = &msg;
}

void DetectorTopic::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    if (m_depthMsg != NULL)
    {
        if ((*m_depthMsg)->header.seq == msg->header.seq)
        {
            //TODO return way point
            m_imageService->perform(msg, *m_depthMsg);
            m_resender.publish(msg);
        }
    }
}



