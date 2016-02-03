/*
 * Teleoperated.cpp
 *
 *  Created on: 13.10.2014
 *      Author: chris
 */

#include "Teleoperated.h"

#include <ros/console.h>
#include <stdlib.h>


#include "../Context.h"

#include "Manipulator.h"


namespace autonohm {

Teleoperated::Teleoperated()
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is Teleoperated.");

   _state_pub           = _nh->advertise<std_msgs::String>("/cortex/state", 1);
   _inspector_state_sub = _nh->subscribe("/inspector_cmd/state", 20, &Teleoperated::inspectorStateCallback, this);


   std_msgs::String msg;
   msg.data = "teleoperated";
   _state_pub.publish(msg);
}

Teleoperated::~Teleoperated()
{
   // nothing to do
}

void Teleoperated::process(void)
{
   // go to next state->explore
   if(_inspector_state.data == "STATE_RELEASED")
   {
      autonohm::Context::getInstance()->setState(new Manipulator());
      delete this;
   }
}

void Teleoperated::inspectorStateCallback(const std_msgs::String& state)
{
   _inspector_state = state;
}

} /* namespace autonohm */
