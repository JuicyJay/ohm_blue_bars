#include "Inspect.h"

#include "../Context.h"
#include "Explore.h"
#include "FoundVictimCandidate.h"

#include <ohm_srvs/NodeControl.h>
#include <ohm_sensor_head/Mode.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace autonohm {

Inspect::Inspect(const geometry_msgs::Quaternion& orientation)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _orientation(orientation),
      _foundVictim(false)
{
    /* Debug publications. */
    ROS_INFO("New state is Inspect.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "inspect";
    _state_pub.publish(msg);


    /* Sensor head control. */
   _srvHeadMode = _nh->serviceClient<ohm_sensor_head::Mode>("/georg/mode");

   ohm_sensor_head::Mode mode;
   mode.request.mode = ohm_sensor_head::Mode::Request::BIND_DIRECTION;

   if (!_srvHeadMode.call(mode))
       ROS_ERROR("Can't call change mode service of the sensor head node.");


   /* Victim Stuff. */
   ohm_srvs::NodeControl control;

   control.request.action = ohm_srvs::NodeControl::Request::START;
   _srvVictimControl = _nh->serviceClient<ohm_srvs::NodeControl>("/georg/victim_detection/control");
   _subVictim = _nh->subscribe("/georg/victim/request", 2, &Inspect::callbackVictim, this);

   if (!_srvVictimControl.call(control))
       ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the victim detection node control service.");


   /* Get the current time stamp for the live time of this state. */
   _stamp = ros::Time::now();
}

Inspect::~Inspect(void)
{
   ohm_srvs::NodeControl control;

   control.request.action = ohm_srvs::NodeControl::Request::PAUSE;

   if (!_srvVictimControl.call(control))
       ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the victim detection node control service.");
}


void Inspect::process(void)
{
    if (_foundVictim)
    {
        ROS_INFO("Found a victim candidate.");

        Context::getInstance()->setState(new FoundVictimCandidate);
        delete this;
        return;
    }

    if ((ros::Time::now() - _stamp).toSec() > 3.0f)
    {
        ROS_INFO("Inspect state already lives 2 seconds. Now its time to kill it.");

	/* Set sensor head mode back to mode NONE. */
        ohm_sensor_head::Mode mode;
        mode.request.mode = ohm_sensor_head::Mode::Request::NONE;

	//        if (!_srvHeadMode.call(mode))
	//           ROS_ERROR("Can't call change mode service of the sensor head node.");

        /* Set the state after and kill myself.*/
        Context::getInstance()->setState(new Explore);
        delete this;
        return;
    }


    geometry_msgs::QuaternionStamped direction;
    static unsigned int seq = 0;

    direction.header.stamp = ros::Time::now();
    direction.header.seq = ++seq;
    direction.header.frame_id = "map";

    direction.quaternion = _orientation;

    _pubDirection.publish(direction);
}

void Inspect::callbackVictim(const ohm_perception::Victim& victims)
{
    _foundVictim = true;
}

} /* namespace autonohm */
