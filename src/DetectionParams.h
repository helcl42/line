/* 
 * File:   DetectionParams.h
 * Author: lubos
 *
 * Created on February 21, 2013, 6:58 PM
 */

#ifndef DETECTIONPARAMS_H
#define	DETECTIONPARAMS_H

struct DetectionParams
{        
    static const unsigned int colorTolerance;
        
    static const unsigned int colorTreshold;
    
    static const unsigned int selectionTreshold;
    
    static float directionDeltaDegrees;
    
    static unsigned int imageWidth;
    
    static unsigned int imageHeight;
    
    static float minLineLengthTreshold;
    
    static float maxLineLengthTreshold;
    
    static float maxStraightnessTreshold;
    
    static float minStraightnessTreshold;
    
    static unsigned int minPointDistance;
    
    static unsigned int inlineTolerance;
    
    static unsigned int maxPointDistance;
    
    static unsigned int noCheckLineBorder;
    
    static unsigned int checkPointSkip;
    
    static unsigned int countOfDirections;            
};


#endif	/* DETECTIONPARAMS_H */

