#include "Inspect.h"

#include "../Context.h"
#include "Explore.h"
#include "FoundVictimCandidate.h"

#include <ohm_srvs/NodeControl.h>
#include <ohm_perception/GetVictim.h>

#include <Eigen/Geometry>

namespace autonohm {

float Inspect::s_inspectionTime = 4.0f;

Inspect::Inspect(const geometry_msgs::Quaternion& orientation)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _foundVictim(false),
      _orientation(orientation)
{
    /* Debug publications. */
    ROS_INFO("New state is Inspect.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "inspect";
    _state_pub.publish(msg);


   /* Victim Stuff. */
   ohm_srvs::NodeControl control;

   control.request.action = ohm_srvs::NodeControl::Request::START;
   _srvVictimControl = _nh->serviceClient<ohm_srvs::NodeControl>("/georg/victim_detection/control");
   _srvVictimStack = _nh->serviceClient<ohm_perception::GetVictim>("/victim/get_victim");

   if (!_srvVictimControl.call(control))
       ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the victim detection node control service.");


   /* Sensor head control. */
    _srvHeadMode = _nh->serviceClient<ohm_actors::SensorHeadMode>("/georg/sensor_head/mode");
    _pubDirection = _nh->advertise<geometry_msgs::QuaternionStamped>("/georg/goal/sensor_head", 2);

    ohm_actors::SensorHeadMode mode;
    mode.request.mode = ohm_actors::SensorHeadMode::Request::BIND_DIRECTION;

    if (!_srvHeadMode.call(mode))
        ROS_ERROR("Can't call change mode service of the sensor head node.");


    /* Get the current time stamp for the live time of this state. */
   _stamp = ros::Time::now();
}

Inspect::~Inspect(void)
{
   ohm_srvs::NodeControl control;

   control.request.action = ohm_srvs::NodeControl::Request::PAUSE;

   if (!_srvVictimControl.call(control))
       ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the victim detection node control service.");


    /* Set sensor head mode back to mode NONE. */
    ohm_actors::SensorHeadMode mode;
    mode.request.mode = ohm_actors::SensorHeadMode::Request::NONE;

    if (!_srvHeadMode.call(mode))
        ROS_ERROR("Can't call change mode service of the sensor head node.");
}


void Inspect::process(void)
{
    /* Victim detection. */
    ohm_perception::GetVictim service;
    service.request.id = ohm_perception::GetVictim::Request::LAST;

    if (_srvVictimStack.call(service))
    {
        ROS_INFO("Found a victim candidate.");

        Context::getInstance()->setState(new FoundVictimCandidate(service.response.victim));
        delete this;
        return;
    }


    /* Publish direction. */
    geometry_msgs::QuaternionStamped direction;
    Eigen::Quaternionf orientation(_orientation.w, _orientation.x, _orientation.y, _orientation.z);
    static unsigned int seq = 0;

    direction.header.stamp = ros::Time::now();
    direction.header.seq = ++seq;
    direction.header.frame_id = "map";

    if ((ros::Time::now() - _stamp).toSec() < s_inspectionTime * 0.5f)
    {
        orientation *= Eigen::AngleAxisf(30.0f * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    }
    else if ((ros::Time::now() - _stamp).toSec() < s_inspectionTime)
    {
        orientation *= Eigen::AngleAxisf(-30.0f * M_PI / 180.0f, Eigen::Vector3f::UnitY());
    }
    else
    {
        ROS_INFO("Inspect state already lives %f seconds. Now its time to kill it.", s_inspectionTime);

        /* Set the state after and kill myself.*/
        Context::getInstance()->setState(new Explore);
        delete this;
        return;
    }

    direction.quaternion.w = orientation.w();
    direction.quaternion.x = orientation.x();
    direction.quaternion.y = orientation.y();
    direction.quaternion.z = orientation.z();

    _pubDirection.publish(direction);
}

} /* namespace autonohm */
