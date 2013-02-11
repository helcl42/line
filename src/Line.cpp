#include "Line.h"

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