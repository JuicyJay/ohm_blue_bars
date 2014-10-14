/*
 * StatePong.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#include <stdlib.h>
#include <ros/console.h>


// ohm cortext includes
#include "../Context.h"
#include "StatePong.h"
#include "StatePing.h"

namespace autonohm
{

StatePong::StatePong(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{

   ROS_INFO("Pong");

}

StatePong::~StatePong(void)
{
   // nothing to
}

void StatePong::process(void)
{
  ROS_DEBUG_STREAM("Pong");

  if(rand()%100<10)
  {
    autonohm::Context::getInstance()->setState(new StatePing());
    delete this;
  }
}

} /* namespace autonohm */
