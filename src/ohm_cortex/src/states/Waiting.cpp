/*
 * Waiting.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Waiting.h"

#include <std_msgs/String.h>

#include "../Context.h"

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



//   if(1/*go back to explore*/) {
//      autonohm::Context::getInstance()->setState(new ConfirmVictim());
//      delete this;
//   }
//
//   if(1/*go back to explore*/) {
//      autonohm::Context::getInstance()->setState(new RejectVictim());
//      delete this;
//   }
//
//   if(1/*go closer to victim*/) {
//      autonohm::Context::getInstance()->setState(new Approach());
//      delete this;
//   }
//
//   if(1/*abort to teleop*/) {
//      autonohm::Context::getInstance()->setState(new Teleoperated());
//      delete this;
//   }
}

} /* namespace autonohm */
