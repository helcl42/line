#include "MoveService.h"

MoveService* MoveService::m_instance = NULL;

MoveService* MoveService::getInstace()
{
    if (m_instance == NULL)
    {
        m_instance = new MoveService();
    }
    return m_instance;
}

bool MoveService::moveTo(Vector2<float>* nextGoal)
{
    if (m_initialized)
    {
        if (!checkSteep(nextGoal))
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

bool MoveService::checkSteep(Vector2<float>* nextGoal)
{
    if(std::abs(m_previousGoal.y - nextGoal->y) < STEP_THRESHOLD)  
    {
       return true;
    }
    return false;
}
