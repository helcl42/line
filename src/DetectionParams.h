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
    
    static const unsigned int directionDeltaDegrees;
    
    static unsigned int imageWidth;
    
    static unsigned int imageHeight;
    
    static unsigned int lineLengthTreshold;                    
    
    static unsigned int straightnessTreshold;
    
    static unsigned int minPointDistance;
    
    static unsigned int inlineTolerance;
    
    static unsigned int maxPointDistance;
    
    static unsigned int noCheckLineBorder;
    
    static unsigned int checkPointSkip;
    
    static unsigned int countOfDirections;        
    
    static void recomputeMetrics(unsigned int w, unsigned int h, unsigned int shrink = 1);
};


#endif	/* DETECTIONPARAMS_H */

