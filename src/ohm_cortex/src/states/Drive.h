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
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include "Explore.h"
#include "Inspect.h"

#include <Eigen/Dense>
#include <limits>

/**
 * @namespace autonohm
 */
namespace autonohm {

namespace drive{
enum enumMode{
   DEFAULT = 0,    //robot will rotate to target orientation
   NO_TARGET_ORI   //robot will not rotate to target orientation -> will rotate to orientation (from before last wp to last wp)
};
}
class Drive : public IState
{
public:
    Drive(const geometry_msgs::Pose& target);
    Drive(const geometry_msgs::Point& target, geometry_msgs::Quaternion& orientation);

    virtual ~Drive(void);
    virtual void process(void);

private: // functions

    //void subPose_callback(const geometry_msgs::PoseStamped& msg);
    void subPath_callback(const nav_msgs::Path& msg);
    void subState_callback(const std_msgs::Bool& msg);

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;
    static ros::Publisher* _pubPath;
    static ros::Publisher* _pubTarget;

    ros::Subscriber _subPath;
    ros::Subscriber _subState;

    geometry_msgs::PoseStamped _targetPose;
    geometry_msgs::Quaternion _targetOrientation;  //just used in non default mode
    nav_msgs::Path _path;

    drive::enumMode _mode;

    bool _old_state;
    bool _reached_target;
    bool _got_path;


};

} // end namespace autonohm

#endif
