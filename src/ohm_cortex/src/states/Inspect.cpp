#include "Inspect.h"

#include "../Context.h"
#include "Explore.h"
#include "FoundVictimCandidate.h"

#include <ohm_srvs/NodeControl.h>
#include <ohm_perception/GetVictim.h>

namespace autonohm {

Inspect::Inspect(void)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _foundVictim(false)
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
   _srvVictimControl = _nh->serviceClient<ohm_srvs::NodeControl>("/victim_detection/control");
   _srvVictimStack = _nh->serviceClient<ohm_perception::GetVictim>("victim/get_victim");

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
    ohm_perception::GetVictim service;
    service.request.id = ohm_perception::GetVictim::Request::LAST;

    if (_srvVictimStack.call(service))
    {
        ROS_INFO("Found a victim candidate.");

        Context::getInstance()->setState(new FoundVictimCandidate(service.response.victim.pose.position));
        delete this;
        return;
    }

    if ((ros::Time::now() - _stamp).toSec() > 2.0f)
    {
        ROS_INFO("Inspect state already lives 2 seconds. Now its time to kill it.");

        /* Set the state after and kill myself.*/
        Context::getInstance()->setState(new Explore);
        delete this;
        return;
    }
}

} /* namespace autonohm */
