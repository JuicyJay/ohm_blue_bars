/*
 * Waiting.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Waiting.h"
#include "Explore.h"
#include "Approach.h"

namespace autonohm {

Waiting::Waiting(void)
{

}

Waiting::~Waiting(void)
{
   // nothing to do
}

void Waiting::process(void)
{



   if(/*go back to explore*/) {
      autonohm::Context::getInstance()->setState(new Explore());
      delete this;
   }

   if(/*go closer to victim*/) {
      autonohm::Context::getInstance()->setState(new Approach());
      delete this;
   }

   if(/*abort to teleop*/) {
      autonohm::Context::getInstance()->setState(new Teleoperated());
      delete this;
   }
}

} /* namespace autonohm */
