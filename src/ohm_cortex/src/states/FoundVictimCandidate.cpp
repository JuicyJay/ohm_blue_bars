#include "FoundVictimCandidate.h"
#include "../Context.h"
#include "Explore.h"

#include <std_msgs/String.h>

namespace autonohm {

FoundVictimCandidate::FoundVictimCandidate(void)
    : _nh(autonohm::Context::getInstance()->getNodeHandle())
{
    /* Debug publications. */
    ROS_INFO("New state is FoundVictimCandidate.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "found victim candidate";
    _state_pub.publish(msg);

    _stamp = ros::Time::now();
}

FoundVictimCandidate::~FoundVictimCandidate(void)
{

}

void FoundVictimCandidate::process(void)
{
    ROS_INFO("Found Victim!!!");

    if ((ros::Time::now() - _stamp).toSec() > 20.0f)
    {
        ROS_INFO("Found Victim Candidate state already lives 20 seconds. Now its time to kill it.");

        /* Set the state after and kill myself.*/
        Context::getInstance()->setState(new Explore);
        delete this;
        return;
    }
}

} // end namespace autonohm

