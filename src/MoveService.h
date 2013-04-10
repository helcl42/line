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
    
public:

    MoveService()
    : m_initialized(false) {}

    ~MoveService() {}

    bool moveTo(Vector2<float>* nextGoal)
    {
        if(m_initialized)
        {
            if(!checkSteep(nextGoal)) 
                return false;
        }
        else
        {
            m_previousGoal.x = nextGoal->x;
            m_previousGoal.y = nextGoal->y;
            m_initialized = true;
        }
                
        MoveBaseClient ac("move_base", true);
        
//        while (!ac.waitForServer(ros::Duration(5.0)))
//        {
//            ROS_INFO("Waiting for the move_base action server to come up");
//        }

        move_base_msgs::MoveBaseGoal goal;

        
        goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = nextGoal->x;
        goal.target_pose.pose.orientation.w = nextGoal->y;
       
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) 
            return false;
        return true;            
    }
    
    bool checkSteep(Vector2<float>* nextGoal)
    {
        //TODO        
        return true;
    }
};

#endif	/* MOVESERVICE_H */

