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
    static unsigned int colorTolerance;
    
    static unsigned int lineLengthTreshold;
    
    static unsigned int colorTreshold;
    
    static unsigned int selectionTreshold;
    
    static unsigned int directionDeltaDegrees;
    
    static unsigned int straightnessTreshold;
    
    static unsigned int minPointDistance;
    
    static unsigned int inlineTolerance;
    
    static unsigned int maxPointDistance;
    
    static unsigned int noCheckLineBorder;
    
    static unsigned int checkPointSkip;
};


#endif	/* DETECTIONPARAMS_H */

