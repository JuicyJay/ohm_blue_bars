/*
 * Manipulator.cpp
 *
 *  Created on: 07.04.2015
 *      Author: chris
 */

#include "Manipulator.h"

#include <ros/console.h>
#include <stdlib.h>


#include "../Context.h"

#include "Teleoperated.h"

namespace autonohm {

Manipulator::Manipulator()
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is Manipulator.");

   _state_pub           = _nh->advertise<std_msgs::String>("state", 1);
   _inspector_state_sub = _nh->subscribe("/inspector_state", 20, &Manipulator::inspectorStateCallback, this);
   _action_sub          = _nh->subscribe("/joy_action",      20, &Manipulator::actionFromJoyCallback, this);

   std_msgs::String msg;
   msg.data = "explore";
   _state_pub.publish(msg);

}

Manipulator::~Manipulator()
{

}


void Manipulator::process(void)
{
   if(_action.status == _action.SWITCH_TELE_ASS)
   {
      ROS_INFO("slow driving enabled");
   }

   if(_inspector_state.data == "STATE_PARKED")
   {
      autonohm::Context::getInstance()->setState(new Teleoperated());
      delete this;
   }
}


void Manipulator::inspectorStateCallback(const std_msgs::String& state)
{
   _inspector_state = state;
}

void Manipulator::actionFromJoyCallback(const ohm_teleop::Action& action)
{
   ROS_INFO("action callback");
   _action = action;
}


} /* namespace autonohm */
