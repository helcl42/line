#include "DetectorTopic.h"

DetectorTopic::DetectorTopic(DetectionSettings* settings)
: m_objectPoint(NULL), m_shrink(0), m_cameraY(0)
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
    double temp = getCameraYPosition(depth);
    if (!isnan(temp) && temp > 0)
    {
        m_cameraY = temp;
    }
    std::cout << "Cam Height!!!!: " << m_cameraY << std::endl;

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
}

void DetectorTopic::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    m_shrink = m_imageService->getShrink();
    m_objectPoint = m_imageService->perform(msg);
    m_resender.publish(msg);

}

double DetectorTopic::getCameraYPosition(const sensor_msgs::Image::ConstPtr& msg)
{
    float lowMidDistance;
    float lowMidDistancePlusHalf;
    double height;
    double temp;

    int countOfOks = 0;
    unsigned int size = (4 * msg->width * msg->height);

    for (int i = 0; i < 3; i++)
    {
        BYTES_TO_FLOAT_L(lowMidDistance, msg->data, size - ((i + 1) * msg->width));
        BYTES_TO_FLOAT_L(lowMidDistancePlusHalf, msg->data, size - (msg->width * msg->height / 2) - ((i + 1) * msg->width));
        temp = getCameraHeight(lowMidDistance, lowMidDistancePlusHalf);
        if (!isnan(temp))
        {
            height += temp;
            countOfOks++;
        }
    }

    if (countOfOks != 0)
    {
        return height / countOfOks;
    }
    else
    {
        return -1;
    }
}

double DetectorTopic::getCameraHeight(float distLow, float distHigh)
{
    float alpha = 6;
    double angle;
    double x1 = distLow * sin(alpha * M_PI / 180);
    double x2 = distLow * cos(alpha * M_PI / 180);

    x2 = distHigh - x2;

    angle = atan(x1 / x2) * 180 / M_PI;

    angle = 90 - angle - alpha;

    return 1.15 * distLow * cos(angle * M_PI / 180) + 0.009 / acos(cos(angle * M_PI / 180));
}

