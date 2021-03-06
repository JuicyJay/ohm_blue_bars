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
#include <ohm_srvs/NodeControl.h>

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
enum enumState{
   WAIT_FOR_SUB = 0,
   PUB_TAEGET,
   WAIT_FOR_PATH,
   PUB_PATH,
   WAIT_FOR_ARIVAL,
   DESTROY,
   NONE
};
}
class Drive : public IState
{
public:
    Drive(const geometry_msgs::Pose& target);
    Drive(const geometry_msgs::Point& target, const geometry_msgs::Quaternion& orientation, IState* stateAfter = 0);

    virtual ~Drive(void);
    virtual void process(void);

private: // functions

    //void subPose_callback(const geometry_msgs::PoseStamped& msg);
    void subPath_callback(const nav_msgs::Path& msg);
    void subState_callback(const std_msgs::Bool& msg);

    bool setEndRotate(bool state);

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;
    static ros::Publisher* _pubPath;
    static ros::Publisher* _pubTarget;

    ros::Subscriber _subPath;
    ros::Subscriber _subState;

    ros::ServiceClient _srv_doendrot;

    geometry_msgs::PoseStamped _targetPose;
    geometry_msgs::Quaternion _targetOrientation;  //just used in non default mode
    nav_msgs::Path _path;

    drive::enumMode _mode;
    drive::enumState _state;

    bool _old_state;
    bool _reached_target;
    bool _got_path;

    IState* _stateAfter;
};

} // end namespace autonohm

#endif
