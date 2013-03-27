/* 
 * File:   Pixel.h
 * Author: lubos
 *
 * Created on January 21, 2013, 6:40 AM
 */

#ifndef PIXEL_H
#define	PIXEL_H

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "../Utils/Utils.h"
#include "IPixel.h"
#include "PixelRGB.h"
#include "PixelXYZ.h"
#include "PixelLUV.h"
#include "../DetectionParams.h"

template <class T> class Pixel;
template <class T> std::ostream& operator<<(std::ostream& out, const Pixel<T>& img);

template <class T>
class Pixel : public IPixel<T>
{
public:
    T r;
    T g;
    T b;
    //T a;

public:

    Pixel()
    : r(0), g(0), b(0) {}

    Pixel(T r, T g, T b, T a = 0)
    : r(r), g(g), b(b)/*, a(a)*/ {}

    virtual ~Pixel() {}       
    
public:
    bool isInInterval(Pixel<T>* rgb1, Pixel<T>* rgb2) const;

    bool hasSimilarColor(Pixel<T>* rgb, unsigned int interval = DetectionParams::colorTolerance) const;

    T& operator[](int index);

    friend std::ostream& operator<<<T>(std::ostream& out, const Pixel<T>& img);
    
public:    
    static double colourProduct(Pixel<T>* rgb1);

    static double colourDifference(Pixel<T>* rgb1, Pixel<T>* rgb2);
    
    static std::string getColorPartString(int index);
    
    static void repairNanValues(Pixel<T>* px);
};


template <class T>
std::string Pixel<T>::getColorPartString(int index)
{
    switch (index)
    {
        case 0:
            return "Red";
        case 1:
            return "Green";
        case 2:
            return "Blue";
        default:
            throw std::runtime_error("Pixel:getColorPartString:InvalidIndex");
    }
}

template <class T>
double Pixel<T>::colourProduct(Pixel<T>* rgb1)
{
    Pixel<T>* luv1 = NULL;
    double prod = 0.0;

    luv1 = rgb1->convertToLUV();

    prod = sqrt(luv1->r * luv1->r + luv1->g * luv1->g + luv1->b * luv1->b);

    SAFE_DELETE(luv1);

    return prod;
}

template <class T>
double Pixel<T>::colourDifference(Pixel<T>* rgb1, Pixel<T>* rgb2)
{
    Pixel<T>* luv1 = NULL;
    Pixel<T>* luv2 = NULL;
    double diff = 0.0;

    luv1 = rgb1->convertToLUV();
    luv2 = rgb2->convertToLUV();

    diff = sqrt(
            (luv1->r - luv2->r) * (luv1->r - luv2->r) +
            (luv1->g - luv2->g) * (luv1->g - luv2->g) +
            (luv1->b - luv2->b) * (luv1->b - luv2->b));

    SAFE_DELETE(luv1);
    SAFE_DELETE(luv2);

    return diff;
}

template <class T>
void Pixel<T>::repairNanValues(Pixel<T>* px)
{
    if (px != NULL)
    {
        if (isnan(px->r))
        {
            px->r = 0;
        }

        if (isnan(px->g))
        {
            px->g = 0;
        }

        if (isnan(px->b))
        {
            px->b = 0;
        }
    }
}

template <class T>
T& Pixel<T>::operator [](int index)
{
    switch (index)
    {
        case 0:
            return r;
        case 1:
            return g;
        case 2:
            return b;
        default:
            throw std::runtime_error("Pixel:operator[]:Invalid index.");
    }
}

/**
 * TODO switched r & b 
 * @param rgb1
 * @param rgb2
 * @return 
 */
template <class T>
bool Pixel<T>::isInInterval(Pixel<T>* rgb1, Pixel<T>* rgb2) const
{
    if (b > rgb2->r || b < rgb1->r)
    {
        return false;
    }

    if (g > rgb2->g || g < rgb1->g)
    {
        return false;
    }

    if (r > rgb2->b || r < rgb1->b)
    {
        return false;
    }
    return true;
}

template <class T>
bool Pixel<T>::hasSimilarColor(Pixel<T>* rgb, unsigned int interval) const
{
    //std::cout << "similar " << *rgb << std::endl;
    if (b + interval > rgb->r && b - interval < rgb->r
            && g + interval > rgb->g && g - interval < rgb->g
            && r + interval > rgb->b && r - interval < rgb->b)
    {
        return true;
    }
    return false;
}

template <class T>
std::ostream& operator<<(std::ostream& out, const Pixel<T>& pix)
{
    out << "{" << pix.r << ",";
    out << pix.g << ",";
    out << pix.b << "}";    
    return out;
}

#endif	/* PIXEL_H */
