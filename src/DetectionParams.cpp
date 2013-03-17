#include <stdexcept>

#include "DetectionParams.h"

/*
 * LineDetection Params
 */

const unsigned int DetectionParams::colorTolerance = 50;

const unsigned int DetectionParams::colorTreshold = 85;

//const unsigned int DetectionParams::selectionTreshold = 20;
float DetectionParams::selectionTreshold = 120;
//shrink 1
float DetectionParams::directionDeltaDegrees = 12; 

float DetectionParams::minLineLengthTreshold = 240;

float DetectionParams::maxLineLengthTreshold = 740;

float DetectionParams::minStraightnessTreshold = 0;

float DetectionParams::maxStraightnessTreshold = 360;

unsigned int DetectionParams::minPointDistance = 16;

unsigned int DetectionParams::maxPointDistance = 1300;

unsigned int DetectionParams::inlineTolerance = 8;

unsigned int DetectionParams::noCheckLineBorder = 80;

unsigned int DetectionParams::checkPointSkip = 24;

unsigned int DetectionParams::countOfDirections = 3; //index = count - 1

unsigned int DetectionParams::imageHeight = 480;

unsigned int DetectionParams::imageWidth = 640;

//shrink 2
//unsigned int DetectionParams::lineLengthTreshold = 120;
//
//unsigned int DetectionParams::straightnessTreshold = 180;
//
//unsigned int DetectionParams::minPointDistance = 8;
//
//unsigned int DetectionParams::maxPointDistance = 650;
//
//unsigned int DetectionParams::inlineTolerance = 4;
//
//unsigned int DetectionParams::noCheckLineBorder = 40;
//
//unsigned int DetectionParams::checkPointSkip = 12;
//
//unsigned int DetectionParams::countOfDirections = 3; //index = count - 1
//
//unsigned int DetectionParams::imageHeight = 480;
//
//unsigned int DetectionParams::imageWidth = 640;

