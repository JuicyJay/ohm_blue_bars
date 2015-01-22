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
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

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

    //void subPose_callback(const geometry_msgs::PoseStamped& msg);
    void subPath_callback(const nav_msgs::Path& msg);
    void subState_callback(const std_msgs::Bool& msg);

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;
    ros::Publisher _pubPath;
    ros::Publisher _pubTarget;

    ros::Subscriber _subPath;
    ros::Subscriber _subState;

    geometry_msgs::PoseStamped _targetPose;
    nav_msgs::Path _path;

    bool _old_state;
    bool _reached_target;
    bool _got_path;

};

} // end namespace autonohm

#endif
