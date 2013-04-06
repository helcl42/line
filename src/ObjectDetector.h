/* 
 * File:   ObjectDetector.h
 * Author: lubos
 *
 * Created on March 6, 2013, 2:33 AM
 */

#ifndef OBJECTDETECTOR_H
#define	OBJECTDETECTOR_H

#include "Pixel/Pixel.h"
#include "DetectionSettings.h"
#include "Line.h"
#include "Image/ImageMap.h"
#include "ImageFilters/Frequency/FFTImageFilterBatch.h"

class ObjectDetector
{
protected:
    ImageMap<float>* m_workImage;

    Image<float>* m_colorImage;

    DetectionColorItem* m_settings;

    PixelRGB<float> m_baseColor;

    FFTImageFilterBatch<float>* m_imageFilterBatch;    

public:       
    ObjectDetector(DetectionColorItem* settings = NULL);
    
    ObjectDetector(ImageMap<float>* image, Image<float>* colorImage);

    virtual ~ObjectDetector();
    
public:
    void setBaseColor();

    PixelRGB<float> getBaseColor() const;

    void setColorSettings(DetectionColorItem* settings);

    DetectionColorItem* getColorSettings() const;

    void setInstance(ImageMap<float>* image, Image<float>* colorImage);    
    
protected:        
    void repaintSimilarColorPlaces(int interval = DetectionParams::colorTolerance);           
        
    void writeLineInImageMap(Line* line, unsigned int val);
    
public:    
    virtual void invalidate() = 0;    
        
    virtual void initDetectionParams(unsigned int shrink = 1) = 0;    
    
protected:    
    virtual bool colorMatch(unsigned int failCount = 0) = 0;
};

#endif	/* OBJECTDETECTOR_H */

