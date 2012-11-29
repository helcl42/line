/* 
 * File:   Pixel.h
 * Author: lubos
 *
 * Created on September 21, 2012, 1:40 PM
 */

#ifndef PIXEL_H
#define PIXEL_H

template <class T>
struct Pixel 
{
    T r;
    T g;
    T b;
    T a;

    Pixel() {}
    Pixel(T r, T g, T b, T a = 0) : r(r), g(g), b(b), a(a) {}

    ~Pixel() {}

    friend std::ostream& operator<<(std::ostream& out, const Pixel & img) 
    {
        out << "{" << img.r << ",";
        out << img.g << ",";
        out << img.b << "}";
        return out;
    }
};



#endif	/* PIXEL_H */

