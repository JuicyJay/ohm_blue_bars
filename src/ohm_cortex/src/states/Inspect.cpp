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
{

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
