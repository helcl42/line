#include "LineDetectorTopic.h"

LineDetectorTopic::LineDetectorTopic(DetectionSettings* settings)
: m_depthMsg(NULL)
{
    m_imageService = new ImageService(settings);
}

LineDetectorTopic::~LineDetectorTopic()
{
    SAFE_DELETE(m_imageService);
}

void LineDetectorTopic::run()
{
    m_sub = m_handler.subscribe("/camera/rgb/image_color", 1, &LineDetectorTopic::imageCallback, this);
    m_subDepth = m_handlerDepth.subscribe("/camera/depth/image", 1, &LineDetectorTopic::depthImageCallback, this);
    m_resender = m_sendHandler.advertise<sensor_msgs::Image > ("resender", 500);
}

void LineDetectorTopic::depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    m_depthMsg = &msg;
}

void LineDetectorTopic::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
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



