#include "LineToCommand.h"
#include "../../../../Vector2.h"
#include "../Arguments/CoordinatePair.h"
#include "../PositionManager.h"

/**
     width = x2 - x1;
     for(int i = 0; i < width; i++) {
         float x = x1 + i;
         float y = y1 + (ratio * i);
         points.add(new Point(x,y));
     }     
 */
void LineToCommand::draw(Line<int>* polygon)
{
    Vector2<float> startPoint = PositionManager::getLastPosition();

    Coordinate* coordX = dynamic_cast<CoordinatePair*> (m_args[0])->GetNumberX();
    Coordinate* coordY = dynamic_cast<CoordinatePair*> (m_args[0])->GetNumberY();   
    
    Vector2<float> endPoint;
    if (m_relative)
    {
        endPoint = Vector2<float>(startPoint.x + coordX->getValue(), startPoint.y + coordY->getValue());
    }
    else
    {
        endPoint = Vector2<float>(coordX->getValue(), coordY->getValue());
    }

    float x, y, ratio, width;
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
        PositionManager::addPosition(endPoint);
    }
    else
    {
        width = (endPoint.y - startPoint.y); //y || line
        if (width > 0)
        {
            for (int i = 0; i < width; i++)
            {
                x = startPoint.x;
                y = startPoint.y + i;
                polygon->addPoint(x, y);
                PositionManager::addPosition(endPoint);
            }
        }
        else
        {
            for (int i = width; i <= 0; i++)
            {
                x = startPoint.x;
                y = endPoint.y - i;
                polygon->addPoint(x, y);
                PositionManager::addPosition(endPoint);
            }
        }
    }
}

void LineToCommand::print()
{
    std::cout << "lineToCommand" << std::endl;
    std::vector<Argument*>::iterator ii;
    for (ii = m_args.begin(); ii != m_args.end(); ++ii)
    {
        (*ii)->print();
    }
}
