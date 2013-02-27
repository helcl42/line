#include <stdexcept>

#include "DetectionParams.h"

/*
 * LineDetection Params
 */

const unsigned int DetectionParams::colorTolerance = 50;

const unsigned int DetectionParams::colorTreshold = 85;

const unsigned int DetectionParams::selectionTreshold = 60;

const unsigned int DetectionParams::directionDeltaDegrees = 12; 


unsigned int DetectionParams::lineLengthTreshold = 120;

unsigned int DetectionParams::straightnessTreshold = 180;

unsigned int DetectionParams::minPointDistance = 8;

unsigned int DetectionParams::maxPointDistance = 650;

unsigned int DetectionParams::inlineTolerance = 4;

unsigned int DetectionParams::noCheckLineBorder = 40;

unsigned int DetectionParams::checkPointSkip = 12;

unsigned int DetectionParams::countOfDirections = 3; //index = count - 1

unsigned int DetectionParams::imageHeight = 480;

unsigned int DetectionParams::imageWidth = 640;


void DetectionParams::recomputeMetrics(unsigned int w, unsigned int h, unsigned int shrink)
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
    
    lineLengthTreshold = imageHeight / (shrink * 1.8);               
    
//    straightnessTreshold = imageHeight / (shrink * 7);
//    
//    minPointDistance = imageHeight / (shrink * 45);
//    
    inlineTolerance = imageHeight / (120 * shrink);
//    
//    maxPointDistance = imageHeight / (1.3714 * shrink);
//    
    noCheckLineBorder = imageHeight / (24 * shrink);
      
    checkPointSkip = imageHeight / (shrink * 18);    
}



/*
 * Rectangle Detection params
 */

//const unsigned int DetectionParams::colorTolerance = 50;
//
//const unsigned int DetectionParams::colorTreshold = 85;
//
//const unsigned int DetectionParams::selectionTreshold = 60;
//
//const unsigned int DetectionParams::directionDeltaDegrees = 2; 
//
//
//unsigned int DetectionParams::lineLengthTreshold = 40;
//
//unsigned int DetectionParams::straightnessTreshold = 6;
//
//unsigned int DetectionParams::minPointDistance = 0;
//
//unsigned int DetectionParams::maxPointDistance = 150;
//
//unsigned int DetectionParams::inlineTolerance = 10;
//
//unsigned int DetectionParams::noCheckLineBorder = 8;
//
//unsigned int DetectionParams::checkPointSkip = 3;
//
//unsigned int DetectionParams::countOfDirections = 1;
//
//unsigned int DetectionParams::imageHeight = 480;
//
//unsigned int DetectionParams::imageWidth = 640;
//
//
//void DetectionParams::recomputeMetrics(unsigned int w, unsigned int h, unsigned int shrink)
//{
//    if(shrink == 0) 
//    {
//        throw std::runtime_error("shrink == 0");
//    }
//   
//    if(w != imageWidth) 
//    {
//        imageWidth = w;
//    }
//    
//    if(h != imageHeight)
//    {
//        imageHeight = h;
//    }
//    
//    lineLengthTreshold = imageHeight / (shrink * 1.8);               
//    
////    straightnessTreshold = imageHeight / (shrink * 7);
////    
////    minPointDistance = imageHeight / (shrink * 45);
////    
//    inlineTolerance = imageHeight / (120 * shrink);
////    
////    maxPointDistance = imageHeight / (1.3714 * shrink);
////    
//    noCheckLineBorder = imageHeight / (24 * shrink);
//      
//    checkPointSkip = imageHeight / (shrink * 18);    
//}
