#include "FoundVictimCandidate.h"
#include "../Context.h"
#include "Explore.h"
#include "Inspect.h"

#include <std_msgs/String.h>
#include <ohm_actors/SensorHeadMode.h>

namespace autonohm {

FoundVictimCandidate::FoundVictimCandidate(const geometry_msgs::Point& goal)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _goal(goal),
      _response(false)
{
    /* Debug publications. */
    ROS_INFO("New state is FoundVictimCandidate.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "found victim candidate";
    _state_pub.publish(msg);


    /* Change mode of the sensor head to look at point. */
    ohm_actors::SensorHeadMode mode;
    mode.request.mode = ohm_actors::SensorHeadMode::Request::LOOK_AT_POINT;
    _srvSensorHeadMode = _nh->serviceClient<ohm_actors::SensorHeadMode>("/georg/sensor_head/mode");
    _subVictimResponse = _nh->subscribe("victim/response", 2, &FoundVictimCandidate::callbackVictimResponse, this);
    _pubGoal = _nh->advertise<geometry_msgs::Point>("/georg/goal/sensor_head", 2);

    if (!_srvSensorHeadMode.call(mode))
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the service sensor head mode.");


    /* Take a time stamp. */
    _stamp = ros::Time::now();
}

FoundVictimCandidate::~FoundVictimCandidate(void)
{

}

void FoundVictimCandidate::process(void)
{
    //    ROS_INFO("Found Victim!!!");

    if ((ros::Time::now() - _stamp).toSec() > 20.0f)
    {
        ROS_INFO("Found Victim Candidate state already lives 20 seconds. Now its time to kill it.");

        /* Set the state after and kill myself.*/
        Context::getInstance()->setState(new Inspect);
        delete this;
        return;
    }
    if (_response)
    {
        ROS_INFO("State FoundVictimCandidate: leaving.");
        Context::getInstance()->setState(new Inspect);
        delete this;
        return;
    }

    _pubGoal.publish(_goal);
}

void FoundVictimCandidate::callbackVictimResponse(const ohm_perception::Victim& msg)
{
    _response = true;
}

} // end namespace autonohm

