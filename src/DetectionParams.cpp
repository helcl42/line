#include "DetectionParams.h"

const unsigned int DetectionParams::colorTolerance = 100;

const unsigned int DetectionParams::colorTreshold = 85;

const unsigned int DetectionParams::selectionTreshold = 35;

const unsigned int DetectionParams::directionDeltaDegrees = 30; 


unsigned int DetectionParams::lineLengthTreshold = 200;

unsigned int DetectionParams::straightnessTreshold = 70;

unsigned int DetectionParams::minPointDistance = 10;

unsigned int DetectionParams::inlineTolerance = 4;

unsigned int DetectionParams::maxPointDistance = 350;

unsigned int DetectionParams::noCheckLineBorder = 20;

unsigned int DetectionParams::checkPointSkip = 12;



void DetectionParams::recomputeMatrics(unsigned int shrink)
{
    lineLengthTreshold = IMAGE_HEIGHT / (shrink + 1);               
    
//    straightnessTreshold = IMAGE_HEIGHT / (shrink * 7);
//    
//    minPointDistance = IMAGE_HEIGHT / (shrink * 45);
//    
//    inlineTolerance = IMAGE_HEIGHT / (120 * shrink);
//    
//    maxPointDistance = IMAGE_HEIGHT / (1.3714 * shrink);
//    
//    noCheckLineBorder = IMAGE_HEIGHT / (24 * shrink);
//        
//    checkPointSkip = IMAGE_HEIGHT / (shrink * 40);    
}


