/*
 * Approach.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Approach.h"

#include "Explore.h"
#include "Inspect.h"

namespace autonohm
{

Approach::Approach(void)
: _nh(autonohm::Context::getInstance()->getNodeHandle())
{

   ROS_INFO("approach");

}

Approach::~Approach(void)
{
   // nothing to do
}

void Approach::process(void)
{

   if(/*no victim found*/) {
      autonohm::Context::getInstance()->setState(new Explore());
      delete this;
   }

   if(/*possible victim approached*/) {
      autonohm::Context::getInstance()->setState(new Inspect());
      delete this;
   }

   if(/*abort to teleop*/) {
      autonohm::Context::getInstance()->setState(new Teleoperated());
      delete this;
   }



}

} /* namespace autonohm */
