/*
 * Waiting.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "Waiting.h"

#include "../Context.h"

#include "Explore.h"
#include "Approach.h"
#include "Teleoperated.h"

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



   if(1/*go back to explore*/) {
      autonohm::Context::getInstance()->setState(new Explore());
      delete this;
   }

   if(1/*go closer to victim*/) {
      autonohm::Context::getInstance()->setState(new Approach());
      delete this;
   }

   if(1/*abort to teleop*/) {
      autonohm::Context::getInstance()->setState(new Teleoperated());
      delete this;
   }
}

} /* namespace autonohm */
