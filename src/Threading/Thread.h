/* 
 * File:    Thread.h
 * Project: ThePlayerServer
 * Author:  lubos
 *
 * Created on May 17, 2012, 11:40 PM
 */

#ifndef THREAD_H
#define	THREAD_H

#include <pthread.h>
#include <cstdlib>
#include <iostream>

#include "../Utils/Utils.h"

class Thread
{
private:
    static void* threadProcedure(void* This)
    {
        ((Thread *) This)->threadProcedure();
        return NULL;
    }

    pthread_t m_thread;
    pthread_attr_t m_attr;
    bool m_bRunning;

protected:
    virtual void threadProcedure() = 0;

public:
    Thread() {}

    virtual ~Thread() {}

    // Returns true if the thread was successfully started
    bool runThread()
    {
        pthread_attr_init(&m_attr);
        pthread_attr_setdetachstate(&m_attr, PTHREAD_CREATE_JOINABLE);
        return (pthread_create(&m_thread, &m_attr, threadProcedure, this) == 0);
    }

    // waits until threadProcedure is finished, true == success_join
    bool waitForThreadToExit()
    {
        int rcCount;
        rcCount = pthread_join(m_thread, NULL);
        pthread_attr_destroy(&m_attr);
        return (rcCount == 0);
    }

    void setRunning(bool r)
    {
        m_bRunning = r;
    }

    bool isRunning() const
    {
        return m_bRunning;
    }
};

#endif	/* THREAD_H */

