/*
 * TargetStack.h
 *
 *  Created on: 17.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#ifndef ___TARGET_STACK_H___
#define ___TARGET_STACK_H___

#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

class TargetStack
{
public:
    static TargetStack* instance(void);

    bool getTarget(geometry_msgs::Pose& pose, unsigned int& id);

private:
    TargetStack(void);

    ros::NodeHandle _nh;
    ros::ServiceClient _srvGetTarget;

    static TargetStack* _instance;
};

#endif
