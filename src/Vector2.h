/* 
 * File:   Vector2.h
 * Author: lubos
 *
 * Created on January 27, 2013, 10:06 AM
 */

#ifndef VECTOR2_H
#define	VECTOR2_H

#include <cmath>
#include <iostream>

template <class Type> class Vector2;
template <class Type> std::ostream& operator<<(std::ostream& out, const Vector2<Type>& vec);     

template <class Type>
class Vector2
{
public:    
    Type x;
    Type y;

public:
    Vector2();

    Vector2(Type x, Type y);

    Vector2(const Vector2& other);

    Vector2& operator=(const Vector2& other);
    
    Vector2& operator+(const Vector2& other);
    
    Vector2& operator+=(const Vector2& other);
    
    Vector2& operator-(const Vector2& other);
    
    Vector2& operator-=(const Vector2& other);
    
    Vector2& operator*(const Vector2& other);    

    Vector2& operator*(float scalar);
    
    bool operator==(const Vector2& other);
    
    bool operator!=(const Vector2& other);
    
    bool operator<(const Vector2& other);
    
    bool operator>(const Vector2& other);    
    
    static Type toRadians(Type val);
    
    static Type toDegrees(Type val);    
    
    Vector2* set(Type x, Type y);    

    Vector2* set(Vector2* other);    

    Vector2* add(Type x, Type y);    

    Vector2* add(Vector2* other);  

    Vector2* sub(Type x, Type y);    

    Vector2* sub(Vector2* other);    

    Vector2* mul(Type scalar);  

    Type length();    

    Vector2* normalize();    

    Type angle();    

    Vector2* rotate(Type angle);    

    double distance(Vector2& other);    

    double distance(Type x, Type y);    

    Type distanceSquared(Vector2& other);
    
    Type distanceSquared(Type x, Type y);    
	
    friend std::ostream& operator<<<Type>(std::ostream& out, const Vector2<Type>& vec);     
};

template<class Type>
Vector2<Type>::Vector2()
      : x(0.0f), y(0.0f) {}

template<class Type>
Vector2<Type>::Vector2(Type x, Type y)
      : x(x), y(y) {}

template<class Type>
Vector2<Type>::Vector2(const Vector2<Type>& other)
{
    this->x = other.x;
    this->y = other.y;
}

template<class Type>
Vector2<Type>& Vector2<Type>::operator=(const Vector2<Type>& other)
{
    if (this != &other)
    {
        this->x = other.x;
        this->y = other.y;
    }
    return (*this);
}

template<class Type>
bool Vector2<Type>::operator==(const Vector2<Type>& other)
{        
    return x == other.x && y == other.y;    
}

template<class Type>
bool Vector2<Type>::operator!=(const Vector2<Type>& other)
{
    return x != other.x || y != other.y;
}

template<class Type>
Vector2<Type>& Vector2<Type>::operator+(const Vector2<Type>& other)
{
    this->x += other.x;
    this->y += other.y;
    return (*this);
}

template<class Type>
Vector2<Type>& Vector2<Type>::operator+=(const Vector2<Type>& other)
{
    this->x += other.x;
    this->y += other.y;
    return (*this);
}

template<class Type>
Vector2<Type>& Vector2<Type>::operator-(const Vector2<Type>& other)
{
    this->x -= other.x;
    this->y -= other.y;
    return (*this);
}

template<class Type>
Vector2<Type>& Vector2<Type>::operator-=(const Vector2<Type>& other)
{
    this->x -= other.x;
    this->y -= other.y;
    return (*this);
}

template<class Type>
Vector2<Type>& Vector2<Type>::operator*(const Vector2<Type>& other)
{
    this->x *= other.x;
    this->y *= other.y;
    return (*this);
}

template<class Type>
Vector2<Type>& Vector2<Type>::operator*(float scalar)
{
    this->x *= scalar;
    this->y *= scalar;
    return (*this);
}

template<class Type>
bool Vector2<Type>::operator<(const Vector2<Type>& other)
{
    return x < other.x && y < other.y;  
}

template<class Type>
bool Vector2<Type>::operator>(const Vector2<Type>& other)
{
    return x > other.x && y > other.y;  
}

template<class Type>
Type Vector2<Type>::toRadians(Type val)
{
    return val * (1 / 180.0f) * M_PI;
}

template<class Type>
Type Vector2<Type>::toDegrees(Type val)
{
    return val * (1 / M_PI) * 180;
}

template<class Type>
Vector2<Type>* Vector2<Type>::set(Type x, Type y)
{
    this->x = x;
    this->y = y;
    return this;
}

template<class Type>
Vector2<Type>* Vector2<Type>::set(Vector2<Type>* other)
{
    this->x = other->x;
    this->y = other->y;
    return this;
}

template<class Type>
Vector2<Type>* Vector2<Type>::add(Type x, Type y)
{
    this->x += x;
    this->y += y;
    return this;
}

template<class Type>
Vector2<Type>* Vector2<Type>::add(Vector2<Type>* other)
{
    this->x += other->x;
    this->y += other->y;
    return this;
}

template<class Type>
Vector2<Type>* Vector2<Type>::sub(Type x, Type y)
{
    this->x -= x;
    this->y -= y;
    return this;
}

template<class Type>
Vector2<Type>* Vector2<Type>::sub(Vector2<Type>* other)
{
    this->x -= other->x;
    this->y -= other->y;
    return this;
}

template<class Type>
Vector2<Type>* Vector2<Type>::mul(Type scalar)
{
    this->x *= scalar;
    this->y *= scalar;
    return this;
}

template<class Type>
Type Vector2<Type>::length()
{
    return sqrt(x * x + y * y);
}

template<class Type>
Vector2<Type>* Vector2<Type>::normalize()
{
    Type len = length();
    if (len != 0)
    {
        this->x /= len;
        this->y /= len;
    }
    return this;
}

template<class Type>
Type Vector2<Type>::angle()
{
    Type angle = Vector2::toDegrees(atan2(y, x));
    if (angle < 0)
    {
        angle += 360;
    }
    return angle;
}

template<class Type>
Vector2<Type>* Vector2<Type>::rotate(Type angle)
{
    Type rad = Vector2::toRadians(angle);
    Type cs = cos(rad);
    Type sn = sin(rad);

    Type newX = x * cs - y * sn;
    Type newY = x * sn + y * cs;

    x = newX;
    y = newY;

    return this;
}

template<class Type>
double Vector2<Type>::distance(Vector2<Type>& other)
{
    Type distX = x - other.x;
    Type distY = y - other.y;
    return (double)sqrt(distX * distX + distY * distY);
}

template<class Type>
double Vector2<Type>::distance(Type x, Type y)
{
    Type distX = this->x - x;
    Type distY = this->y - y;
    return (double)sqrt(distX * distX + distY * distY);
}

template<class Type>
Type Vector2<Type>::distanceSquared(Vector2<Type>& other)
{
    float distX = x - other.x;
    float distY = y - other.y;
    return distX * distX + distY * distY;
}

template<class Type>
Type Vector2<Type>::distanceSquared(Type x, Type y)
{
    Type distX = this->x - x;
    Type distY = this->y - y;
    return distX * distX + distY * distY;
}

template<class Type>
std::ostream& operator<<(std::ostream& out, const Vector2<Type>& vec)
{
    out << "(" << vec.x << ", " << vec.y << ")";
    return out;
}

#endif	/* VECTOR2_H */

