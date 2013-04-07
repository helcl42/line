#include "CloseCommand.h"
#include "../Arguments/CoordinatePair.h"
#include "../PositionManager.h"

void CloseCommand::draw(Line* polygon)
{
    Vector2<float> startPoint = PositionManager::getLastBasePointPosition();
    Vector2<float> endPoint(polygon->getEnd().x, polygon->getEnd().y);
    float width, ratio, x, y;

    if (endPoint.x != startPoint.x)
    {
        ratio = (endPoint.y - startPoint.y) / (endPoint.x - startPoint.x);
        width = endPoint.x - startPoint.x;

        if (width > 0)
        {
            for (int i = 0; i < width; i++)
            {
                x = startPoint.x + i;
                y = startPoint.y + (ratio * i);
                polygon->addPoint(x, y);
            }
        }
        else
        {
            for (int i = width; i <= 0; i++)
            {
                x = startPoint.x + i;
                y = startPoint.y + (ratio * i);
                y = y < 0 ? -y : y;
                polygon->addPoint(x, y);
            }
        }
    }
    else
    {
        width = (endPoint.y - startPoint.y); //y ax
        if (width > 0)
        {
            for (int i = 0; i < width; i++)
            {
                x = startPoint.x;
                y = startPoint.y + i;
                polygon->addPoint(x, y);
            }
        }
        else
        {
            for (int i = width; i <= 0; i++)
            {
                x = startPoint.x;
                y = startPoint.y + i;
                polygon->addPoint(x, y);
            }
        }
    }
}

void CloseCommand::print()
{
    std::cout << "closeCommand" << std::endl;
}
