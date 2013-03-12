/* 
 * File:   ObjectDetector.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:33 AM
 */

#ifndef OBJECTDETECTOR_H
#define	OBJECTDETECTOR_H

#include "Pixel.h"
#include "DetectionSettings.h"
#include "ImageFilter.h"
#include "EdgeFilterStrategy.h"

class ObjectDetector
{
protected:
    Image<float>* m_workImage;

    Image<float>* m_colorImage;

    DetectionColorItem* m_settings;

    PixelRGB<float> m_baseColor;

    ImageFilter<float>* m_imageFilter;

    EdgeFilterStrategy<float>* m_edgeFilter;
    

public:       
    ObjectDetector(DetectionColorItem* settings = NULL);
    
    ObjectDetector(Image<float>* image, Image<float>* colorImage);

    virtual ~ObjectDetector();
    
public:
    void setBaseColor();

    PixelRGB<float> getBaseColor() const;

    void setColorSettings(DetectionColorItem* settings);

    DetectionColorItem* getColorSettings() const;

    void setImages(Image<float>* image, Image<float>* colorImage);    
    
public:    
    virtual void invalidate() = 0;    
        
    virtual void initDetectionParams(unsigned int shrink = 1) = 0;
    
protected:        
    void repaintSimilarColorPlaces(int interval = DetectionParams::colorTolerance);   
    
    virtual bool colorMatch(unsigned int failCount = 0) = 0;
};

#endif	/* OBJECTDETECTOR_H */

