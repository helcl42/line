#include "Timer.h"

Timer::Timer(bool resetAfterLog)
: m_startTimeInMicroSec(0.0), m_endTimeInMicroSec(0.0),
  m_resetAfterLog(resetAfterLog), m_interval(0.0), m_stopped(false)
{
    m_startCount.tv_sec = m_startCount.tv_usec = 0;
    m_endCount.tv_sec = m_endCount.tv_usec = 0;   
}

Timer::Timer(double interval, bool resetAfterLog)
: m_startTimeInMicroSec(0.0), m_endTimeInMicroSec(0.0),
  m_resetAfterLog(resetAfterLog), m_interval(interval), m_stopped(false)
{
    m_startCount.tv_sec = m_startCount.tv_usec = 0;
    m_endCount.tv_sec = m_endCount.tv_usec = 0;   
}

void Timer::start()
{
    m_stopped = false; // reset stop flag

    gettimeofday(&m_startCount, NULL);
}

void Timer::stop()
{
    m_stopped = true; // set timer stopped flag

    gettimeofday(&m_endCount, NULL);
}

bool Timer::isStarted() const
{
    return !m_stopped;
}

bool Timer::isElapsedMs(unsigned int ms)
{
    if(m_stopped)
    {
        return false; //not stared yet
    }
    
    if(((m_endTimeInMicroSec - m_startTimeInMicroSec) * 0.001) > ms)
    {
        return true;
    }
    return false;
}

bool Timer::isElapsedS(unsigned int s)
{
    if(m_stopped)
    {
        return false;
    }
    
    if(((m_endTimeInMicroSec - m_startTimeInMicroSec) * 0.0000001) > s)
    {
        return true;
    }
    return false;
}

double Timer::getFPS()
{
    return 1 / getElapsedTimeInSec();
}

double Timer::getElapsedTimeInMicroSec()
{
    if (!m_stopped)
    {
        gettimeofday(&m_endCount, NULL);
    }

    m_startTimeInMicroSec = (m_startCount.tv_sec * 1000000.0) + m_startCount.tv_usec;
    m_endTimeInMicroSec = (m_endCount.tv_sec * 1000000.0) + m_endCount.tv_usec;

    return m_endTimeInMicroSec - m_startTimeInMicroSec;
}

double Timer::getElapsedTimeInMilliSec()
{
    return this->getElapsedTimeInMicroSec() * 0.001;
}

double Timer::getElapsedTimeInSec()
{
    return this->getElapsedTimeInMicroSec() * 0.000001;
}

void Timer::logTime() 
{
    std::stringstream ss;
    double res = getElapsedTimeInMicroSec();
    int sec = res / 1000000;
    res -= sec * 1000000;
    ss << sec << "s ";    
    sec = res / 1000;
    res -= sec * 1000;
    ss << sec << "ms ";    
    ss << res << " us";    
    std::cout << ss.str() << std::endl;
}

void Timer::logTimeUs() 
{
    std::stringstream ss;
    double res = getElapsedTimeInMicroSec();
    int sec = res / 1000000;
    res -= sec * 1000000;
    if(sec > 0) ss << sec;    
    sec = res / 1000;
    res -= sec * 1000;
    if(sec > 0) ss << sec;    
    ss << res;      
    std::cout << ss.str() << std::endl;
}
