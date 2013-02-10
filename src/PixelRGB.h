/* 
 * File:   PixelRGB.h
 * Author: lubos
 *
 * Created on January 21, 2013, 6:43 AM
 */

#ifndef PIXELRGB_H
#define	PIXELRGB_H

#include "Pixel.h"

template <class T> class PixelLUV;
template <class T> class PixelXYZ;
template <class T> class PixelRGB;

template <class T> std::ostream& operator<<(std::ostream& out, const PixelRGB<T>& pix);

template<class T>
class PixelRGB : public Pixel<T>
{
public:
    PixelRGB()
    : Pixel<T>() {}

    PixelRGB(T r, T g, T b)
    : Pixel<T>(r, g, b) {}

    virtual ~PixelRGB() {}

    Pixel<T>* convertToRGB();

    Pixel<T>* convertToLUV();

    Pixel<T>* convertToXYZ();

    friend std::ostream& operator<<<T>(std::ostream& out, const PixelRGB<T>& pix);
};

template <class T>
Pixel<T>* PixelRGB<T>::convertToRGB()
{
    PixelRGB<T>* rgb = new PixelRGB<T > ();
    rgb->r = this->r;
    rgb->g = this->g;
    rgb->b = this->b;
    return rgb;
}

template <class T>
Pixel<T>* PixelRGB<T>::convertToLUV()
{
    Pixel<T>* xyzPixel = NULL;
    Pixel<T>* luvPixel = NULL;

    xyzPixel = this->convertToXYZ();            
    luvPixel = xyzPixel->convertToLUV();    
    Pixel<T>::repairNanValues(luvPixel);
    SAFE_DELETE(xyzPixel);
    return luvPixel;
}

template <class T>
Pixel<T>* PixelRGB<T>::convertToXYZ()
{
    PixelXYZ<T>* xyz = new PixelXYZ<T > ();
    PixelRGB<T> temp;

    if (this->r <= 0.04045)
    {
        temp.r = this->r / 12.92;
    }
    else
    {
        temp.r = pow((this->r + 0.055) / 1.055, 2.4);
    }

    if (this->g <= 0.04045)
    {
        temp.g = this->g / 12.92;
    }
    else
    {
        temp.g = pow((this->g + 0.055) / 1.055, 2.4);
    }

    if (this->b <= 0.04045)
    {
        temp.b = this->b / 12.92;
    }
    else
    {
        temp.b = pow((this->b + 0.055) / 1.055, 2.4);
    }

    xyz->r = 0.4124 * temp.r + 0.3576 * temp.g + 0.1805 * temp.b;
    xyz->g = 0.2126 * temp.r + 0.7152 * temp.g + 0.0722 * temp.b;
    xyz->b = 0.0193 * temp.r + 0.1192 * temp.g + 0.9505 * temp.b;
    return xyz;
}

template <class T>
std::ostream& operator<<(std::ostream& out, const PixelRGB<T>& pix)
{
    out << "RGB(" << pix.r << ",";
    out << pix.g << ",";
    out << pix.b << ")";
    return out;
}

#endif	/* PIXELRGB_H */

