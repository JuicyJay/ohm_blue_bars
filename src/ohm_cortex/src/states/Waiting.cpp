/*
 * Waiting.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris and Knueppl
 */

#include "Waiting.h"
#include "Explore.h"

#include <std_msgs/String.h>

#include "../Context.h"
#include "../GetTransformation.h"

#include "ConfirmVictim.h"
#include "RejectVictim.h"
#include "Approach.h"
#include "Teleoperated.h"

namespace autonohm {

Waiting::Waiting(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("waiting");

   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   std_msgs::String msg;
   msg.data = "waiting";
   _state_pub.publish(msg);
}

Waiting::~Waiting(void)
{
   // nothing to do
}

void Waiting::process(void)
{
    GetTransformation* listener = GetTransformation::instance();

    if (!listener->waitAndLookUpTransform("map", "simon/base_footprint", 2.0f))
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": look up transform failed.");
        return;
    }

    const float distance = listener->position().norm();

    ROS_INFO("Current distance of simon is %f.", distance);

    if (distance >= 3.0f)
    {
        ROS_INFO("Will leave state Waiting.");
        Context::getInstance()->setState(new Explore);
        delete this;
    }
}

} /* namespace autonohm */
