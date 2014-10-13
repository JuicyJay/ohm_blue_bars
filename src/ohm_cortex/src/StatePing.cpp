/*
 * StatePing.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */
#include <stdlib.h>
#include <ros/console.h>

#include "Context.h"
#include "StatePing.h"
#include "StatePong.h"

namespace autonohm
{

StatePing::StatePing()
{
   ROS_INFO("Ping");
}

StatePing::~StatePing()
{

}

void StatePing::process()
{
  ROS_DEBUG_STREAM("Ping");

  if(rand()%100<10)
  {
    autonohm::Context::getInstance()->setState(new StatePong());
    delete this;
  }
}

} /* namespace autonohm */
