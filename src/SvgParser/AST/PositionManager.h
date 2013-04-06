/* 
 * File:   RelativePosition.h
 * Author: lubos
 *
 * Created on March 24, 2013, 4:54 PM
 */

#ifndef POSITIONFACTORY_H
#define	POSITIONFACTORY_H

#include "../../Vector2.h"
#include "Arguments/CoordinatePair.h"

class PositionManager
{
private:
    static std::vector<Vector2<float> > m_basePoints;
    
    static std::vector<Vector2<float> > m_positions;
    
    static unsigned int m_positionsCounter;
    
public:                    
    static void addPosition(Vector2<int> coordiantePair);
    
    static void addPosition(Vector2<float> coordiantePair);            
    
    static void addPosition(CoordinatePair* pair);
    
    static Vector2<float> getFirstPosition();
    
    static Vector2<float> getLastPosition();
    
    static Vector2<float> getPositionAt(int index);    
    
    
    static void addBasePoint(Vector2<int> coordiantePair);
    
    static void addBasePoint(Vector2<float> coordiantePair);            
    
    static void addBasePoint(float x, float y);
    
    static void addBasePoint(CoordinatePair* pair);
    
    static Vector2<float> getFirstBasePointPosition();
    
    static Vector2<float> getLastBasePointPosition();
    
    static Vector2<float> getBasePointPositionAt(int index);    
};

#endif	/* RELATIVEPOSITION_H */
