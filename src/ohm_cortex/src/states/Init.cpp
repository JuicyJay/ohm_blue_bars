/*
 * Init.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Init.h"

#include "../Context.h"
#include "Explore.h"

// ros includes
#include <std_msgs/String.h>
#include "StatePing.h"

namespace autonohm {

Init::Init(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is Init.");

   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   std_msgs::String msg;
   msg.data = "init";

   _state_pub.publish(msg);
}

Init::~Init(void)
{
   // nothing to do
}

void Init::process(void)
{
    Context::getInstance()->setState(new Explore);
    delete this;
}

} /* namespace autonohm */
