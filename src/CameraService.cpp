#include "CameraService.h"

double CameraService::getCameraYPosition(const sensor_msgs::Image::ConstPtr& msg)
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

double CameraService::getCameraHeight(float distLow, float distHigh)
{
    float alpha = 6;
    double angle;
    double x1 = distLow * sin(alpha * M_PI / 180);
    double x2 = distLow * cos(alpha * M_PI / 180);

    x2 = distHigh - x2;

    angle = atan(x1 / x2) * 180 / M_PI;
    
    return 1.17 * distHigh * sin(angle * M_PI / 180) + 0.01 * acos(cos(angle / 2 * M_PI / 180));
    //angle = 90 - angle - alpha;    
    //return 1.15 * distLow * cos(angle * M_PI / 180) + 0.009 / acos(cos(angle * M_PI / 180));            
}


