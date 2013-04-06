#include "CurveToCommand.h"
#include "../../../Vector2.h"
#include "../Arguments/CoordinatePair.h"
#include "../PositionManager.h"

void CurveToCommand::draw(Line* polygon)
{
    CoordinatePair* tempCoords;

    for (int i = 0; i < m_args.size(); i += 3)
    {
        Vector2<float> startPoint = PositionManager::getLastPosition();
        std::vector<Vector2<float> > tempLine;
        tempLine.push_back(Vector2<float>(startPoint.x, startPoint.y));

        for (int j = i; j < i + 3; j++)
        {         
            tempCoords = dynamic_cast<CoordinatePair*> (m_args[j]);
            if (tempCoords != NULL)
            {
                if (m_relative)
                {
                    tempLine.push_back(Vector2<float>(tempCoords->GetNumberX()->getValue() + startPoint.x, tempCoords->GetNumberY()->getValue() + startPoint.y));
                }
                else
                {
                    tempLine.push_back(Vector2<float>(tempCoords->GetNumberX()->getValue(), tempCoords->GetNumberY()->getValue()));
                }
            }
            else
            {
                throw std::runtime_error("CurveTo: absolutePath -> Invalid argument.");
            }
        }       

        std::vector<Vector2<float> > bezierResult = getBezierApproximation(tempLine, 256);

        for (unsigned int i = 0; i < bezierResult.size(); i++)
        {
            polygon->addPoint(bezierResult[i].x, bezierResult[i].y);
        }
                
        PositionManager::addPosition(tempLine.back());        
    }
}

void CurveToCommand::print()
{
    std::string rel = m_relative ? "relative" : "absolute";
    std::cout << "curveToCommand: " << rel << std::endl;
    std::vector<Argument*>::iterator ii;
    for (ii = m_args.begin(); ii != m_args.end(); ii++)
    {
        (*ii)->print();
    }
}

std::vector<Vector2<float> > CurveToCommand::getBezierApproximation(std::vector<Vector2<float> >& line, int outputSegmentCount)
{
    std::vector<Vector2<float> > points;
    for (int i = 0; i <= outputSegmentCount; i++)
    {
        double t = (double) i / outputSegmentCount;
        points.push_back(getBezierPoint(t, line, 0, line.size()));
    }
    return points;
}

Vector2<float> CurveToCommand::getBezierPoint(double t, std::vector<Vector2<float> >& line, int index, int count)
{
    if (count == 1)
    {
        return line[index];
    }

    Vector2<float> p1 = getBezierPoint(t, line, index, count - 1);
    Vector2<float> p2 = getBezierPoint(t, line, index + 1, count - 1);
    
    return Vector2<float>((1 - t) * p1.x + t * p2.x, (1 - t) * p1.y + t * p2.y);
}
