#include "Line.h"

/**
 * 
 * @param img
 */
void Line::writeToMessage(const sensor_msgs::Image::ConstPtr& img)
{
    if (img->width > 0 && img->height > 0)
    {
        unsigned char* temp;

        std::vector<Vector2<int> >::iterator ii;
        for (ii = points.begin(); ii != points.end(); ++ii)
        {
            int index = (*ii).y * img->width + (*ii).x;
            for (int i = 0; i < 3; i++)
            {
                if (i == 0)
                {
                    temp = (unsigned char*) &img->data[index];
                    *temp = 255;
                }
                else
                {
                    temp = (unsigned char*) &img->data[index + i];
                    *temp = 0;
                }
            }
        }
    }
}

Line::Line(Line* input)
{
    std::vector<Vector2<int> >::const_iterator ii;
    for (ii = input->points.begin(); ii != input->points.end(); ++ii)
    {
        points.push_back(*ii);
    }
}

double Line::getDirection()
{
    double direction = 0.0;
    
    Vector2<int> tempBeginPoint = points.front();
    Vector2<int> tempEndPoint = points.back();
        
    direction = (double) (tempEndPoint.y - tempBeginPoint.y) / (double) (tempEndPoint.x - tempBeginPoint.x);
    
    return direction;
}

unsigned int Line::getStraightestIndex(Line** lines)
{
    float minDistance = 100000.0;
    float innerMaxDistance = 0;
    float distance = 0;
    unsigned int index = 0;
    unsigned int len = 0;
    Line* temp = NULL;

    for (unsigned int i = 0; i < 4; i++)
    {
        temp = lines[i];
        len = temp->points.size();
        Vector2<float> n(temp->points.back().y - temp->points.front().y, -(temp->points.back().x - temp->points.front().x));
        float c = -n.x * temp->points.front().x - n.y * temp->points.front().y;
        float unsq = sqrt(n.x * n.x + n.y * n.y);

        for (unsigned int j = 0; j < len; j++)
        {
            distance = (n.x * temp->points[j].x + n.y * temp->points[j].y + c) / unsq;
            if (distance > innerMaxDistance)
            {
                innerMaxDistance = distance;
            }
        }

        if (innerMaxDistance < minDistance)
        {
            minDistance = innerMaxDistance;
            index = i;
        }
    }
    return index;
}

void Line::computeStraightnessFactor()
{
    float maxDistance = 0;
    float distance = 0;    

    if (points.size() > 0)
    {        
        Vector2<float> n(points.back().y - points.front().y, -(points.back().x - points.front().x));
        float c = -n.x * points.back().x - n.y * points.front().y;

        for (unsigned int j = 0; j < points.size(); j++)
        {
            distance = (n.x * points[j].x + n.y * points[j].y + c) / sqrt(n.x * n.x + n.y * n.y);
            if(distance < 0)
            {
                distance = -distance;
            }
            
            if (distance > maxDistance)
            {
                maxDistance = distance;
            }
        }

        straightnessFactor = maxDistance;
    }
}

unsigned int Line::getMaxLengthIndex(Line** lines)
{
    unsigned int maxLength = 0;
    unsigned int maxIndex = 0;

    for (unsigned int i = 0; i < 4; i++)
    {
        if (lines[i]->points.size() > maxLength)
        {
            maxLength = lines[i]->points.size();
            maxIndex = i;
        }
    }
    return maxIndex;
}

bool Line::isSimilar(Line* input)
{
    for (unsigned int i = 0; i < points.size(); ++i)
    {
        for (unsigned int j = 0; j < input->points.size(); j++)
        {
            if (points[i] == input->points[j])
            {
                //std::cout << "similar line " << points[i] << " " << input->points[j] << std::endl;
                return true;
            }
        }
    }
    //std::cout << "NOT similar" << std::endl;
    return false;
}

/**
 * 
 * @param out
 * @param line
 * @return 
 */
std::ostream& operator<<(std::ostream& out, const Line& line)
{
    out << "Line len = " << line.points.size() << std::endl;

    std::vector<Vector2<int> >::const_iterator ii;
    for (ii = line.points.begin(); ii != line.points.end(); ++ii)
    {
        out << *ii << " ";
    }
    return out;
}