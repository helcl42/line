/* 
 * File:   MoveService.h
 * Author: lubos
 *
 * Created on April 8, 2013, 3:24 PM
 */

#ifndef MOVESERVICE_H
#define	MOVESERVICE_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "Vector2.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveService
{
private:
    Vector2<float> m_previousGoal;

    bool m_initialized;

    static MoveService* m_instance;
    
private:    
    MoveService()
    : m_initialized(false) {}
    
public:
    static MoveService* getInstace();

    ~MoveService() {}

    bool moveTo(Vector2<float>* nextGoal);
    
    bool checkSteep(Vector2<float>* nextGoal);
};

#endif	/* MOVESERVICE_H */

