#include "Line.h"

/**
 * 
 * @param input
 */
Line::Line(Line* input)
{
    std::vector<Vector2<int> >::const_iterator ii;
    for (ii = input->points.begin(); ii != input->points.end(); ++ii)
    {
        points.push_back(*ii);
    }
}

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

/**
 * 
 * @param line1
 * @param line2
 * @return 
 */
Vector2<int> Line::getDirection(Line* line1, Line* line2)
{
    return Vector2<int>(0, 0);
}

/**
 * 
 * @param lines
 * @return 
 */
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

/**
 * 
 * @param input
 * @return 
 */
bool Line::isSimilar(Line* input)
{
    for (unsigned int i = 0; i < points.size(); ++i)
    {
        for (unsigned int j = 0; j < input->points.size(); j++)
        {
            if (points[i] == input->points[j])
            {                
                return true;
            }
        }
    }    
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