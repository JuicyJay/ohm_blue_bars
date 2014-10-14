/*
 * ConfirmVictim.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#include "ConfirmVictim.h"

#include "../Context.h"

#include "Explore.h"


namespace autonohm {

ConfirmVictim::ConfirmVictim(void)
{

}

ConfirmVictim::~ConfirmVictim(void)
{
   // nothing to do
}

void ConfirmVictim::process(void)
{
   // add victim to map


   // confirm in gui


   // go to next state->explore
   autonohm::Context::getInstance()->setState(new Explore());
   delete this;
}

} /* namespace autonohm */
