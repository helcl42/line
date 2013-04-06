/* 
 * File:   IImageFilter.h
 * Author: lubos
 *
 * Created on April 3, 2013, 2:00 PM
 */

#ifndef IIMAGEFILTER_H
#define	IIMAGEFILTER_H

template <class T>
class IImageFilter
{
public:
    virtual void applyFilter() = 0;
};

#endif	/* IIMAGEFILTER_H */

