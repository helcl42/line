/* 
 * File:   ObjectDetector.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:33 AM
 */

#ifndef ABSTRACTDETECTOR_H
#define	ABSTRACTDETECTOR_H

#include "Pixel/Pixel.h"
#include "DetectionSettings.h"
#include "Polygon.h"
#include "Image/ImageMap.h"
#include "ImageFilters/Frequency/FFTImageFilterBatch.h"
#include "Shapes/ShapesCache.h"

class AbstractDetector
{
protected:
    ImageMap<float>* m_workImage;

    Image<float>* m_colorImage;

    DetectionColorItem* m_settings;

    PixelRGB<float> m_baseColor;

    FFTImageFilterBatch<float>* m_imageFilterBatch;    
    
public:       
    AbstractDetector(DetectionColorItem* settings = NULL);
    
    AbstractDetector(ImageMap<float>* image, Image<float>* colorImage);

    virtual ~AbstractDetector();
    
public:
    void setBaseColor();

    PixelRGB<float> getBaseColor() const;

    void setColorSettings(DetectionColorItem* settings);

    DetectionColorItem* getColorSettings() const;

    void setInstance(ImageMap<float>* image, Image<float>* colorImage);    
    
protected:        
    void repaintSimilarColorPlaces(int interval = DetectionParams::colorTolerance);           
        
    void writeLineInImageMap(Polygon<int>* line, unsigned int val);
    
public:    
    virtual void invalidate() = 0;    
        
    virtual void initDetectionParams(unsigned int shrink = 1) = 0;    
    
protected:    
    virtual bool colorMatch(unsigned int failCount = 0) = 0;
};

#endif	/* ABSTRACTDETECTOR_H */

