/* 
 * File:   Timer.h
 * Author: lubos
 *
 * Created on January 22, 2013, 10:37 PM
 */

#ifndef TIMER_H
#define	TIMER_H

#include <cstdlib>
#include <sys/time.h>
#include <iostream>
#include <sstream>

class Timer
{
private:
    double m_startTimeInMicroSec;
    
    double m_endTimeInMicroSec;    
    
    bool m_resetAfterLog;
    
    double m_interval;

    timeval m_startCount;
    
    timeval m_endCount;
    
    bool m_stopped;

public:    
    
    Timer(bool resetAfterLog = true);
    
    Timer(double interval, bool resetAfterLog = true);
    
    ~Timer() {}

public:
    void start();
    
    void stop();
    
    bool isElapsedMs(unsigned int ms);
    
    bool isElapsedS(unsigned int s);
    
    double getFPS();
    
    bool isStarted() const;
    
    double getElapsedTimeInSec();
    
    double getElapsedTimeInMilliSec();
    
    double getElapsedTimeInMicroSec();
    
    void logTime();
    
    void logTimeUs();
};
#endif	/* TIMER_H */

