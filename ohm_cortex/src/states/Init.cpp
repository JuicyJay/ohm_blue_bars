/*
 * Init.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Init.h"

#include "../Context.h"
#include "Explore.h"
#include "Waiting.h"

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
    ::sleep(5);

  //    Context::getInstance()->setState(new Explore);
    Context::getInstance()->setState(new Waiting);
    delete this;
}

} /* namespace autonohm */
