#include "LookAtTarget.h"
#include "Inspect.h"
#include "../Context.h"

#include <ohm_actors/SensorHeadMode.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace autonohm {

LookAtTarget::LookAtTarget(const geometry_msgs::Quaternion& orientation)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _orientation(orientation)
{
    /* Debug publications. */
    ROS_INFO("New state is Inspect.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "look to target";
    _state_pub.publish(msg);


    /* Sensor head control. */
    _pubDirection = _nh->advertise<geometry_msgs::QuaternionStamped>("/georg/goal/sensor_head", 2);
    _srvHeadMode = _nh->serviceClient<ohm_actors::SensorHeadMode>("/georg/sensor_head/mode");

    ohm_actors::SensorHeadMode mode;
    mode.request.mode = ohm_actors::SensorHeadMode::Request::BIND_DIRECTION;

    if (!_srvHeadMode.call(mode))
        ROS_ERROR("Can't call change mode service of the sensor head node.");


    /* Get the current time stamp for the live time of this state. */
    _stamp = ros::Time::now();
}

LookAtTarget::~LookAtTarget(void)
{
    /* Set sensor head mode back to mode NONE. */
    ohm_actors::SensorHeadMode mode;
    mode.request.mode = ohm_actors::SensorHeadMode::Request::NONE;

    //        if (!_srvHeadMode.call(mode))
    //           ROS_ERROR("Can't call change mode service of the sensor head node.");
}

void LookAtTarget::process(void)
{
    if ((ros::Time::now() - _stamp).toSec() > 1.0f)
    {
        ROS_INFO("LookAtTarget state already lives 1 seconds. Now its time to kill it.");
        /* Set the state after and kill myself.*/
	//        Context::getInstance()->setState(new Inspect);
	//        delete this;

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

} // end namespace autonohm
