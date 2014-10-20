/*
 * Explore.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Explore.h"

#include "../Context.h"

namespace autonohm {

Explore::Explore(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("Ping");

   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   std_msgs::String msg;
   msg.data = "explore";
   _state_pub.publish(msg);
}


Explore::~Explore(void)
{

}

void Explore::process(void)
{

}

} /* namespace autonohm */
