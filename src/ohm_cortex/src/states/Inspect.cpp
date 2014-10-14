/*
 * Inspekt.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "../Inspect.h"

#include "Waiting.h"

namespace autonohm {

Inspect::Inspect(void)
{

}

Inspect::~Inspect(void) {
}


void Inspect::process(void)
{


   if(/*abort to teleop*/) {
      autonohm::Context::getInstance()->setState(new Teleoperated());
      delete this;
   }
}

} /* namespace autonohm */
