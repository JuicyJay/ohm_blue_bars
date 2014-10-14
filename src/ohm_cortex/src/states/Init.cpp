/*
 * Init.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Init.h"

#include "../Context.h"

// ros includes
#include <std_msgs/String.h>
#include "StatePing.h"

namespace autonohm {

Init::Init(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("Init");

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
   ROS_DEBUG_STREAM("Init");

   if(rand()%100<10)
   {
     autonohm::Context::getInstance()->setState(new StatePing());
     delete this;
   }
}

} /* namespace autonohm */
