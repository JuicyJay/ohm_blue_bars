/*
 * Inspekt.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Inspect.h"

#include "Waiting.h"
#include "Teleoperated.h"

#include "../Context.h"


namespace autonohm {

Inspect::Inspect(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("Ping");

   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   std_msgs::String msg;
   msg.data = "inspect";
   _state_pub.publish(msg);
}

Inspect::~Inspect(void)
{

}


void Inspect::process(void)
{
   if(1/*request to to user*/) {
      autonohm::Context::getInstance()->setState(new Waiting());
      delete this;
   }

   if(1/*abort to teleop*/) {
      autonohm::Context::getInstance()->setState(new Teleoperated());
      delete this;
   }
}

} /* namespace autonohm */
