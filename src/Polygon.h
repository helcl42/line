/* 
 * File:   Line.h
 * Author: lubos
 *
 * Created on February 10, 2013, 1:21 AM
 */

#ifndef LINE_H
#define	LINE_H


#include <vector>
#include <stdexcept>
#include <cstdlib>
#include "Vector2.h"
#include "DetectionParams.h"

template <class T> class Polygon;
template <class T> std::ostream& operator<<(std::ostream& out, const Polygon<T>& line);

template <class T>
class Polygon
{
private:
    std::vector<Vector2<T> > points;

    bool locked;

    double straightnessFactor;

    double direction;

    double directionDegrees;

    double length;

public:
    Polygon()
    : locked(false), straightnessFactor(0), direction(0), directionDegrees(0), length(0) {}

    Polygon(const Polygon<T>& input, unsigned int skip = 1);

    Polygon(Polygon<T>* input, unsigned int skip = 1);

    virtual ~Polygon() {}

    bool isClose(Polygon<T>* input, unsigned int crossCount = 0);

    bool isInline(Polygon<T>* input);

    bool isTooNarrow();

    void addPoint(Vector2<T> pt);

    void addPoint(T x, T y);

    double getDistanceTo(Vector2<T>& point);

    void computeProperties();

    void deletePoints();
    
    bool isBeginNearer(Polygon<T>* other);

    Vector2<T>* getPointPtr(unsigned int index);

    Vector2<T> getPoint(unsigned int index);

    void setPoint(Vector2<T> point, unsigned int index);

    void setPoint(T x, T y, unsigned int index);

    unsigned int getSize() const;

    friend std::ostream& operator<<<T> (std::ostream& out, const Polygon<T>& line);

public:
    static Polygon<T>* getMaxLengthLine(Polygon<T>** lines);

    static Polygon<T>* getStraightesstLine(Polygon<T>** lines);


public: //should be private
    double computeStraightnessFactor();

    double computeDirectionInDegrees();
        
    double computeDirectionDegreesTo(unsigned int indexTo, bool fromBegin = true) const;

    double computeDirection();

    double computeLength();

public:
    unsigned int getCountOfPoints() const;

    Vector2<T> getBegin();

    Vector2<T> getEnd();
    
    Vector2<T>* getBeginPtr();

    Vector2<T>* getEndPtr();

    double getDirection() const;

    void setDirection(double direction);

    double getDirectionDegrees() const;    

    void setDirectionDegrees(double directionDegrees);        

    double getLength() const;

    void setLength(double length);

    bool isLocked() const;

    void setLocked(bool locked);

    double getStraightnessFactor() const;

    void setStraightnessFactor(double straightnessFactor);
};

template <class T>
Polygon<T>::Polygon(const Polygon<T>& input, unsigned int skip)
{
    typename std::vector<Vector2<T> >::const_iterator ii;
    for (ii = input.points.begin(); ii != input.points.end(); ii += skip)
    {
        points.push_back(*ii);
    }
    computeProperties();
}

template <class T>
Polygon<T>::Polygon(Polygon* input, unsigned int skip)
{
    if (skip < 0) skip = 1;

    for (unsigned int i = 0; i < input->getSize(); i += skip)
    {
        points.push_back(input->getPoint(i));
    }
    computeProperties();
}

template <class T>
void Polygon<T>::computeProperties()
{
    length = computeLength();
    direction = computeDirection();
    directionDegrees = computeDirectionInDegrees();
    straightnessFactor = computeStraightnessFactor();
}

template <class T>
double Polygon<T>::computeDirection()
{
    Vector2<T> tempBeginPoint = points.front();
    Vector2<T> tempEndPoint = points.back();

    return (double) (tempEndPoint.y - tempBeginPoint.y) / (double) (tempEndPoint.x - tempBeginPoint.x);
}

template <class T>
double Polygon<T>::computeDirectionDegreesTo(unsigned int indexTo, bool fromBegin) const
{   
    const Vector2<T>* tempBeginPoint = NULL;
    const Vector2<T>* tempEndPoint = NULL;
    
    if(fromBegin) 
    {
        tempBeginPoint = &points.front();
    }
    else
    {
        tempBeginPoint = &points.back();
    }
    
    tempEndPoint = &points[indexTo];

    return atan2(tempEndPoint->y - tempBeginPoint->y, tempEndPoint->x - tempBeginPoint->x) * 180 / M_PI;
}

template <class T>
double Polygon<T>::computeDirectionInDegrees()
{
    Vector2<T> tempBeginPoint = points[DetectionParams::noCheckLineBorder];
    Vector2<T> tempEndPoint = points[points.size() - DetectionParams::noCheckLineBorder];

    return atan2(tempEndPoint.y - tempBeginPoint.y, tempEndPoint.x - tempBeginPoint.x) * 180 / M_PI;
}

template <class T>
double Polygon<T>::computeLength()
{
    double length = 0.0;

    Vector2<T> tempBeginPoint = points.front();
    Vector2<T> tempEndPoint = points.back();

    length = (double) (tempEndPoint.y - tempBeginPoint.y) * (tempEndPoint.y - tempBeginPoint.y)
            + (double) (tempEndPoint.x - tempBeginPoint.x) * (tempEndPoint.x - tempBeginPoint.x);
    return pow(length, 0.5);
}

template <class T>
double Polygon<T>::computeStraightnessFactor()
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

template <class T>
Polygon<T>* Polygon<T>::getStraightesstLine(Polygon<T>** lines)
{
    double minDistance = 100000.0;
    double distance = 0;
    Polygon<T>* straightest = NULL;

    for (unsigned int i = 0; i < 8; i++)
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

template <class T>
bool Polygon<T>::isBeginNearer(Polygon<T>* other) 
{
    Vector2<T>* thisBegin = getBeginPtr();
    
    if(thisBegin->distance(other->getBeginPtr()) < thisBegin->distance(other->getEndPtr())) 
    {
        return true;
    }
    return false;
}

template <class T>
Polygon<T>* Polygon<T>::getMaxLengthLine(Polygon<T>** lines)
{
    double maxLength = 0;
    double tempLength;
    Polygon<T>* longest = NULL;

    for (unsigned int i = 0; i < 8; i++)
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

template <class T>
void Polygon<T>::deletePoints()
{
    points.clear();
}

template <class T>
unsigned int Polygon<T>::getSize() const
{
    return points.size();
}

template <class T>
void Polygon<T>::addPoint(Vector2<T> point)
{
    points.push_back(point);
}

template <class T>
void Polygon<T>::addPoint(T x, T y)
{
    points.push_back(Vector2<T > (x, y));
}

template <class T>
void Polygon<T>::setPoint(Vector2<T> point, unsigned int index)
{
    if (index >= 0 && index < points.size())
    {
        points[index] = point;
    }
    else
    {
        throw std::runtime_error("Line:setPoint -> invalid index");
    }
}

template <class T>
void Polygon<T>::setPoint(T x, T y, unsigned int index)
{
    if (index >= 0 && index < points.size())
    {
        points[index] = Vector2<int>(x, y);
    }
    else
    {
        throw std::runtime_error("Line:setPoint -> invalid index");
    }
}

template <class T>
Vector2<T> Polygon<T>::getPoint(unsigned int index)
{
    if (index >= 0 && index < points.size())
    {
        return points[index];
    }
    else
    {
        throw std::runtime_error("Line:getPoint -> invalid index");
    }
}

template <class T>
Vector2<T>* Polygon<T>::getPointPtr(unsigned int index)
{
    if (index >= 0 && index < points.size())
    {
        return &points[index];
    }
    else
    {
        throw std::runtime_error("Line:getPointPtr -> invalid index");
    }
}

template <class T>
bool Polygon<T>::isInline(Polygon<T>* input)
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
        if (result + DetectionParams::inlineTolerance > q && result - DetectionParams::inlineTolerance < q)
        {
            return true;
        }
    }
    return false;
}

template <class T>
bool Polygon<T>::isClose(Polygon<T>* input, unsigned int crossCount)
{
    double distance = 0;
    unsigned int foundCrosses = 0;

    for (unsigned int i = DetectionParams::noCheckLineBorder; i < points.size() - DetectionParams::noCheckLineBorder; i += DetectionParams::checkPointSkip)
    {
        for (unsigned int j = DetectionParams::noCheckLineBorder; j < input->points.size() - DetectionParams::noCheckLineBorder; j += DetectionParams::checkPointSkip)
        {
            distance = points[i].distance(input->points[j]);
            if (distance < DetectionParams::minPointDistance || distance > DetectionParams::maxPointDistance)
            {
                foundCrosses++;
                if (foundCrosses >= crossCount)
                {
                    return true;
                }
            }
        }
    }
    return false;
}

template <class T>
unsigned int Polygon<T>::getCountOfPoints() const
{
    return points.size();
}

template <class T>
bool Polygon<T>::isTooNarrow()
{
    float h = pow(2, 0.5) * length / 5.5;
    if (straightnessFactor >= h)
    {
        return true;
    }
    return false;
}

template <class T>
double Polygon<T>::getDistanceTo(Vector2<T>& point)
{
    Vector2<float> n(-(points.back().y - points.front().y), (points.back().x - points.front().x));
    int c = -(n.x * points.front().x) - (n.y * points.front().y);

    return (double) (n.x * point.x + n.y * point.y + c) / (double) sqrt(n.x * n.x + n.y * n.y);
}

template <class T>
Vector2<T>* Polygon<T>::getBeginPtr()
{
    return &points.front();
}

template <class T>
Vector2<T>* Polygon<T>::getEndPtr()
{
    return &points.back();
}

template <class T>
Vector2<T> Polygon<T>::getBegin()
{
    return points.front();
}

template <class T>
Vector2<T> Polygon<T>::getEnd()
{
    return points.back();
}

template <class T>
double Polygon<T>::getDirection() const
{
    return direction;
}

template <class T>
void Polygon<T>::setDirection(double direction)
{
    this->direction = direction;
}

template <class T>
double Polygon<T>::getDirectionDegrees() const
{
    return directionDegrees;
}

template <class T>
void Polygon<T>::setDirectionDegrees(double directionDegrees)
{
    this->directionDegrees = directionDegrees;
}

template <class T>
double Polygon<T>::getLength() const
{
    return length;
}

template <class T>
void Polygon<T>::setLength(double length)
{
    this->length = length;
}

template <class T>
bool Polygon<T>::isLocked() const
{
    return locked;
}

template <class T>
void Polygon<T>::setLocked(bool locked)
{
    this->locked = locked;
}

template <class T>
double Polygon<T>::getStraightnessFactor() const
{
    return straightnessFactor;
}

template <class T>
void Polygon<T>::setStraightnessFactor(double straightnessFactor)
{
    this->straightnessFactor = straightnessFactor;
}

template <class T>
std::ostream& operator<<(std::ostream& out, const Polygon<T>& line)
{
    out << "Line len = " << line.length << " Straightness = " << line.straightnessFactor << std::endl;

    typename std::vector<Vector2<T> >::const_iterator ii;
    for (ii = line.points.begin(); ii != line.points.end(); ++ii)
    {
        out << *ii << " ";
    }
    return out;
}

#endif	/* LINE_H */

