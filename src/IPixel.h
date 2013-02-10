/* 
 * File:   IPixel.h
 * Author: lubos
 *
 * Created on January 21, 2013, 4:40 PM
 */

#ifndef IPIXEL_H
#define	IPIXEL_H

template <class T> class Pixel;

template <class T>
class IPixel 
{
public:    
    virtual Pixel<T>* convertToRGB() = 0;
    
    virtual Pixel<T>* convertToLUV() = 0;
    
    virtual Pixel<T>* convertToXYZ() = 0;
};


#endif	/* IPIXEL_H */

