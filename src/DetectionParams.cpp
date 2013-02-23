#include <stdexcept>

#include "DetectionParams.h"

const unsigned int DetectionParams::colorTolerance = 100;

const unsigned int DetectionParams::colorTreshold = 85;

const unsigned int DetectionParams::selectionTreshold = 25;

const unsigned int DetectionParams::directionDeltaDegrees = 33; 


unsigned int DetectionParams::lineLengthTreshold = 130;

unsigned int DetectionParams::straightnessTreshold = 140;

unsigned int DetectionParams::minPointDistance = 6;

unsigned int DetectionParams::maxPointDistance = 650;

unsigned int DetectionParams::inlineTolerance = 4;

unsigned int DetectionParams::noCheckLineBorder = 40;

unsigned int DetectionParams::checkPointSkip = 12;

unsigned int DetectionParams::imageHeight = 480;

unsigned int DetectionParams::imageWidth = 640;


void DetectionParams::recomputeMatrics(unsigned int w, unsigned int h, unsigned int shrink)
{
    if(shrink == 0) 
    {
        throw std::runtime_error("shrink == 0");
    }
   
    if(w != imageWidth) 
    {
        imageWidth = w;
    }
    
    if(h != imageHeight)
    {
        imageHeight = h;
    }
    
    lineLengthTreshold = imageHeight / (shrink + 1);               
    
    straightnessTreshold = imageHeight / (shrink * 7);
    
    minPointDistance = imageHeight / (shrink * 45);
    
    inlineTolerance = imageHeight / (120 * shrink);
    
    maxPointDistance = imageHeight / (1.3714 * shrink);
    
    noCheckLineBorder = imageHeight / (24 * shrink);
        
    checkPointSkip = imageHeight / (shrink * 40);    
}


