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


#include "Explore.h"


namespace autonohm {

Teleoperated::Teleoperated()
{

}

Teleoperated::~Teleoperated()
{
   // nothing to do
}

void Teleoperated::process(void)
{
   // go to next state->explore
   if(1)
   {
      autonohm::Context::getInstance()->setState(new Explore());
      delete this;
   }
}

} /* namespace autonohm */
