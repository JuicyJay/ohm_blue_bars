/*
 * StatePing.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */
#include <stdlib.h>
#include <ros/console.h>

#include "../Context.h"
#include "StatePing.h"
#include "StatePong.h"

#include <std_msgs/String.h>

namespace autonohm
{

StatePing::StatePing(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("Ping");

   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   std_msgs::String msg;
   msg.data = "init";
   _state_pub.publish(msg);
}

StatePing::~StatePing(void)
{

}

void StatePing::process(void)
{
  ROS_DEBUG_STREAM("Ping");

//  std_msgs::String msg;
//  msg.data = "ping";
//  _state_pub.publish(msg);

  if(rand()%100<10)
  {
    autonohm::Context::getInstance()->setState(new StatePong());
    delete this;
  }
}

} /* namespace autonohm */
