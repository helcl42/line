#include <stdexcept>

#include "DetectionParams.h"

const unsigned int DetectionParams::colorTolerance = 100;

const unsigned int DetectionParams::colorTreshold = 85;

const unsigned int DetectionParams::selectionTreshold = 25;

const unsigned int DetectionParams::directionDeltaDegrees = 32; 


unsigned int DetectionParams::lineLengthTreshold = 130;

unsigned int DetectionParams::straightnessTreshold = 120;

unsigned int DetectionParams::minPointDistance = 6;

unsigned int DetectionParams::maxPointDistance = 650;

unsigned int DetectionParams::inlineTolerance = 4;

unsigned int DetectionParams::noCheckLineBorder = 40;

unsigned int DetectionParams::checkPointSkip = 12;



void DetectionParams::recomputeMatrics(unsigned int shrink)
{
    if(shrink == 0) 
    {
        throw std::runtime_error("shrink == 0");
    }
    
    lineLengthTreshold = IMAGE_HEIGHT / (shrink + 1);               
    
    straightnessTreshold = IMAGE_HEIGHT / (shrink * 7);
    
    minPointDistance = IMAGE_HEIGHT / (shrink * 45);
    
    inlineTolerance = IMAGE_HEIGHT / (120 * shrink);
    
    maxPointDistance = IMAGE_HEIGHT / (1.3714 * shrink);
    
    noCheckLineBorder = IMAGE_HEIGHT / (24 * shrink);
        
    checkPointSkip = IMAGE_HEIGHT / (shrink * 40);    
}


