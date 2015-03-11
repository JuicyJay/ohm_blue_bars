#include "Inspect.h"

#include "../Context.h"
#include "Explore.h"

#include <ohm_sensor_head/Mode.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace autonohm {

Inspect::Inspect(const geometry_msgs::Quaternion& orientation)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _orientation(orientation)
{
    /* Debug publications. */
    ROS_INFO("New state is Inspect.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "inspect";
    _state_pub.publish(msg);


    /* Sensor head control. */
   _srvHeadMode = _nh->serviceClient<ohm_sensor_head::Mode>("/georg/mode");
   _pubDirection = _nh->advertise<geometry_msgs::QuaternionStamped>("/georg/goal/sensor_head", 2);

   ohm_sensor_head::Mode mode;
   mode.request.mode = ohm_sensor_head::Mode::Request::BIND_DIRECTION;

   if (!_srvHeadMode.call(mode))
       ROS_ERROR("Can't call change mode service of the sensor head node.");


   _stamp = ros::Time::now();
}

Inspect::~Inspect(void)
{

}


void Inspect::process(void)
{
    if ((ros::Time::now() - _stamp).toSec() > 2.0f)
    {
        ROS_INFO("Inspect state alrady lives 2 seconds. Now its time to kill it.");

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

    ROS_INFO("Inspect: publish direction.");
    _pubDirection.publish(direction);
}

} /* namespace autonohm */
