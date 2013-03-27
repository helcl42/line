/* 
 * File:   PixelXYZ.h
 * Author: lubos
 *
 * Created on January 21, 2013, 6:47 AM
 */

#ifndef PIXELXYZ_H
#define	PIXELXYZ_H

#include "Pixel.h"

template <class T> class PixelRGB;
template <class T> class PixelLUV;
template <class T> class PixelXYZ;

template <class T> std::ostream& operator<<(std::ostream& out, const PixelXYZ<T>& pix);

template<class T>
class PixelXYZ : public Pixel<T>
{
public:
    PixelXYZ()
    : Pixel<T>() {}

    PixelXYZ(T r, T g, T b)
    : Pixel<T>(r, g, b) {}

    virtual ~PixelXYZ() {}

    Pixel<T>* convertToRGB();

    Pixel<T>* convertToLUV();

    Pixel<T>* convertToXYZ();

    friend std::ostream& operator<<<T>(std::ostream& out, const PixelXYZ<T>& pix);
};

template <class T>
Pixel<T>* PixelXYZ<T>::convertToRGB()
{
    PixelRGB<T>* rgb = new PixelRGB<T > ();
    double x1, y2, z3, r1, g2, b3;

    x1 = this->r / 100; //X from 0 to  95.047
    y2 = this->g / 100; //Y from 0 to 100.000
    z3 = this->b / 100; //Z from 0 to 108.883

    r1 = x1 * 3.2406 + y2 * (-1.5372) + z3 * (-0.4986);
    g2 = x1 * -0.9689 + y2 * 1.8758 + z3 * 0.0415;
    b3 = x1 * 0.0557 + y2 * (-0.2040) + z3 * 1.0570;

    if (r1 > 0.0031308)
    {
        r1 = 1.055 * pow(r1, (1 / 2.4)) - 0.055;
    }
    else
    {
        r1 = 12.92 * r1;
    }

    if (g2 > 0.0031308)
    {
        g2 = 1.055 * pow(g2, (1 / 2.4)) - 0.055;
    }
    else
    {
        g2 = 12.92 * g2;
    }

    if (b3 > 0.0031308)
    {
        b3 = 1.055 * pow(b3, (1 / 2.4)) - 0.055;
    }
    else
    {
        b3 = 12.92 * b3;
    }

    rgb->r = r1 * 255;
    rgb->g = g2 * 255;
    rgb->b = b3 * 255;
    return rgb;
}

template <class T>
Pixel<T>* PixelXYZ<T>::convertToLUV()
{
    PixelLUV<T>* luv = new PixelLUV<T > ();

    double Xr = 1.0 / 3;
    double Yr = 1.0 / 3;
    double Zr = 1.0 / 3;

    double yr, us, vs, usr, vsr, e, k;

    yr = this->g / Yr;
    us = 4 * this->r / (this->r + 15 * this->g + 3 * this->b);
    vs = 9 * this->g / (this->r + 15 * this->g + 3 * this->b);

    usr = 4 * Xr / (Xr + 15 * Yr + 2 * Zr);
    vsr = 9 * Yr / (Xr + 15 * Yr + 3 * Zr);
    e = 216.0 / 24389;
    k = 24389.0 / 27;

    if (yr > e)
    {
        luv->r = 116 * pow(yr, 1.0 / 3) - 16;
    }
    else
    {
        luv->r = k * yr;
    }

    luv->g = 13.0 * luv->r * (us - usr);
    luv->b = 13.0 * luv->r * (vs - vsr);
    return luv;
}

template <class T>
Pixel<T>* PixelXYZ<T>::convertToXYZ()
{
    PixelXYZ<T>* xyz = new PixelXYZ<T > ();
    xyz->r = this->r;
    xyz->g = this->g;
    xyz->b = this->b;
    return xyz;
}

template <class T>
std::ostream& operator<<(std::ostream& out, const PixelXYZ<T>& pix)
{
    out << "XYZ(" << pix.r << ",";
    out << pix.g << ",";
    out << pix.b << ")";
    return out;
}

#endif	/* PIXELXYZ_H */

