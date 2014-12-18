/*
 * Drive.h
 *
 *  Created on: 14.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#ifndef ___DRIVE_H___
#define ___DRIVE_H___

#include "../IState.h"
#include "../Context.h"

#include <ros/ros.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <ohm_path_plan/MoveToAction.h>

#include "Explore.h"

/**
 * @namespace autonohm
 */
namespace autonohm {

class Drive : public IState
{
public:
    Drive(const geometry_msgs::Pose& target);

    virtual ~Drive(void);
    virtual void process(void);

private: // functions

    void subPose_callback(const geometry_msgs::PoseStamped& msg);

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;

    ros::Subscriber _subPose;
    actionlib::SimpleActionClient<ohm_path_plan::MoveToAction>* _ac;

    geometry_msgs::PoseStamped _currentPose;
    geometry_msgs::PoseStamped _targetPose;

    bool _pose_rdy;
};

} // end namespace autonohm

#endif
