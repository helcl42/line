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

Line::Line(const Line& input)
{
    std::vector<Vector2<int> >::const_iterator ii;
    for (ii = input.points.begin(); ii != input.points.end(); ++ii)
    {
        points.push_back(*ii);
    }
    computeProperties();
}

void Line::computeProperties()
{
    length = computeLength();
    direction = computeDirection();
    directionDegrees = computeDirectionInDegrees();
    straightnessFactor = computeStraightnessFactor();
}

double Line::computeDirection()
{
    Vector2<int> tempBeginPoint = points.front();
    Vector2<int> tempEndPoint = points.back();

    return (double) (tempEndPoint.y - tempBeginPoint.y) / (double) (tempEndPoint.x - tempBeginPoint.x);
}

double Line::computeDirectionInDegrees()
{
    Vector2<int> tempBeginPoint = points.front();
    Vector2<int> tempEndPoint = points.back();

    return atan2(tempEndPoint.y - tempBeginPoint.y, tempEndPoint.x - tempBeginPoint.x) * 180 / M_PI;
}

double Line::computeLength()
{
    double length = 0.0;

    Vector2<int> tempBeginPoint = points.front();
    Vector2<int> tempEndPoint = points.back();

    length = (double) (tempEndPoint.y - tempBeginPoint.y) * (tempEndPoint.y - tempBeginPoint.y)
            + (double) (tempEndPoint.x - tempBeginPoint.x) * (tempEndPoint.x - tempBeginPoint.x);
    return pow(length, 0.5);
}

double Line::computeStraightnessFactor()
{
    double maxDistance = 0;
    double distance = 0;

    if (points.size() > 0)
    {
        Vector2<float> n(-(points.back().y - points.front().y), (points.back().x - points.front().x));
        int c = -(n.x * points.front().x) - (n.y * points.front().y);
        double div = sqrt(n.x * n.x + n.y * n.y);

        for (unsigned int j = 0; j < points.size(); j++)
        {
            distance = (n.x * points[j].x + n.y * points[j].y + c) / div;
            if (distance < 0)
            {
                distance = -distance;
            }

            if (isnan(distance)) continue;

            if (distance > maxDistance)
            {
                maxDistance = distance;
            }
        }
    }    
    return maxDistance;
}

Line* Line::getStraightesstLine(Line** lines)
{
    double minDistance = 100000.0;
    double distance = 0;
    Line* straightest = NULL;

    for (unsigned int i = 0; i < 4; i++)
    {
        if (lines[i] != NULL)
        {
            distance = lines[i]->computeStraightnessFactor();

            if (distance < minDistance)
            {
                minDistance = distance;
                straightest = lines[i];
            }
        }
    }
    return straightest;
}

Line* Line::getMaxLengthLine(Line** lines)
{
    double maxLength = 0;
    double tempLength;
    Line* longest = NULL;

    for (unsigned int i = 0; i < 4; i++)
    {
        if (lines[i] != NULL)
        {
            tempLength = lines[i]->computeLength();
            
            if (tempLength > maxLength)
            {
                maxLength = tempLength;
                longest = lines[i];
            }
        }
    }
    return longest;
}

bool Line::isInline(Line* input)
{
    double q = points.front().y - points.front().x * direction;
    unsigned int testedIndex;
    double result;

    if (input->points.back().x > points.front().x && input->points.back().x > points.back().x
            && input->points.front().x > points.front().x && input->points.front().x > points.front().x)
    {
        return true;
    }

    if (input->points.back().y > points.front().y && input->points.back().y > points.back().y
            && input->points.front().y > points.front().y && input->points.front().y > points.front().y)
    {
        return true;
    }

    for (int i = 0; i < 20; i++)
    {
        testedIndex = rand() % input->points.size();
        result = input->points[testedIndex].y - input->points[testedIndex].x * direction;
        if (result + INLINE_TOLERANCE > q && result - INLINE_TOLERANCE < q)
        {            
            return true;
        }

    }
    return false;
}

bool Line::isClose(Line* input)
{
    double distance = 0;

    for (unsigned int i = NO_CHECK_LINE_BORDER; i < points.size() - NO_CHECK_LINE_BORDER; i += CHECK_POINT_SKIP)
    {
        for (unsigned int j = NO_CHECK_LINE_BORDER; j < input->points.size() - NO_CHECK_LINE_BORDER; j += CHECK_POINT_SKIP)
        {
            distance = points[i].distance(input->points[j]);
            if (distance < MIN_POINT_DISTANCE || distance > MAX_POINT_DISTANCE)
            {     
                return true;
            }
        }
    }    
    return false;
}

std::ostream& operator<<(std::ostream& out, const Line& line)
{
    out << "Line len = " << line.length << std::endl;

    std::vector<Vector2<int> >::const_iterator ii;
    for (ii = line.points.begin(); ii != line.points.end(); ++ii)
    {
        out << *ii << " ";
    }
    return out;
}
