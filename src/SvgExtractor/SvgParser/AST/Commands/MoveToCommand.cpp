#include "MoveToCommand.h"

MoveToCommand::MoveToCommand()
: m_startPoint(NULL)
{
}

MoveToCommand::~MoveToCommand()
{
}

void MoveToCommand::draw(Polygon<int>* polygon)
{
    if (m_relative)
    {
        Vector2<float> prevBase = PositionManager::getLastBasePointPosition();
        Coordinate* coordX = m_startPoint->GetNumberX();
        Coordinate* coordY = m_startPoint->GetNumberY();

        PositionManager::addBasePoint(prevBase.x + coordX->getValue(), prevBase.y + coordY->getValue());
        std::cout << "RELATIVE base" << std::endl;
    }
    else
    {
        PositionManager::addBasePoint(m_startPoint);
    }
}

void MoveToCommand::print()
{
    std::cout << "MoveToCommand " << m_args.size() << std::endl;

    std::vector<Argument*>::iterator ii;
    for (ii = m_args.begin(); ii != m_args.end(); ++ii)
    {
        (*ii)->print();
    }
}

CoordinatePair* MoveToCommand::getStartPoint() const
{
    return m_startPoint;
}

void MoveToCommand::setStartPoint(CoordinatePair* startPoint)
{
    this->m_startPoint = startPoint;
}
