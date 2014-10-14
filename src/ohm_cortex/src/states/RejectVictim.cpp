/*
 * RejectVictim.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "RejectVictim.h"

#include "../Context.h"

#include "Explore.h"

namespace autonohm {

RejectVictim::RejectVictim()
{

}

RejectVictim::~RejectVictim()
{
   // nothing to do
}

void RejectVictim::process(void)
{
   // blacklist victim

   // show rejection in gui


   // go to next state->explore
   autonohm::Context::getInstance()->setState(new Explore());
   delete this;
}


} /* namespace autonohm */
