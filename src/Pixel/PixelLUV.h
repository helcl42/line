/* 
 * File:   PixelLUV.h
 * Author: lubos
 *
 * Created on January 21, 2013, 6:49 AM
 */

#ifndef PIXELLUV_H
#define	PIXELLUV_H

#include "Pixel.h"

template <class T> class PixelXYZ;
template <class T> class PixelRGB;
template <class T> class PixelLUV;

template <class T> std::ostream& operator<<(std::ostream& out, const PixelLUV<T>& pix);

template<class T>
class PixelLUV : public Pixel<T>
{
public:

    PixelLUV()
    : Pixel<T>() {}

    PixelLUV(T r, T g, T b)
    : Pixel<T>(r, g, b) {}

    virtual ~PixelLUV() {}

    Pixel<T>* convertToRGB();

    Pixel<T>* convertToLUV();

    Pixel<T>* convertToXYZ();

    friend std::ostream& operator<<<T>(std::ostream& out, const PixelLUV<T>& pix);
};

template <class T>
Pixel<T>* PixelLUV<T>::convertToRGB()
{
    Pixel<T>* xyzPixel = NULL;
    Pixel<T>* rgbPixel = NULL;
    xyzPixel = convertToXYZ();
    rgbPixel = xyzPixel->convertToRGB();
    SAFE_DELETE(xyzPixel);
    return rgbPixel;
}

template <class T>
Pixel<T>* PixelLUV<T>::convertToLUV()
{
    PixelLUV<T>* luv = new PixelLUV<T > ();
    luv->r = this->r;
    luv->g = this->g;
    luv->b = this->b;
    return luv;
}

template <class T>
Pixel<T>* PixelLUV<T>::convertToXYZ()
{
    double rx = 95.047; //Observer= 2°, Illuminant= D65
    double ry = 100.000;
    double rz = 108.883;
    double y1, ru, rv;

    PixelXYZ<T>* xyz = new PixelXYZ<T > ();

    y1 = (this->r + 16) / 116;
    if (pow(this->g, 3) > 0.008856)
    {
        y1 = pow(y1, 3);
    }
    else
    {
        y1 = (y1 - 16 / 116) / 7.787;
    }

    ru = (4 * rx) / (rx + (15 * ry) + (3 * rz));
    rv = (9 * ry) / (rx + (15 * ry) + (3 * rz));

    ru = this->g / (13 * this->r) + ru;
    rv = this->b / (13 * this->r) + rv;

    xyz->r = y1 * 100;
    xyz->g = -(9 * this->g * ru) / ((ru - 4) * rv - ru * rv);
    xyz->b = (9 * this->g - (15 * rv * this->g) - (rv * this->r)) / (3 * rv);
    return xyz;
}

template <class T>
std::ostream& operator<<(std::ostream& out, const PixelLUV<T>& pix)
{
    out << "LUV(" << pix.r << ",";
    out << pix.g << ",";
    out << pix.b << ")";
    return out;
}

#endif	/* PIXELLUV_H */

