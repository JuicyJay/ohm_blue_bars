/*
 * ConfirmVictim.h
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_CONFIRMVICTIM_H_
#define OHM_CORTEX_SRC_STATES_CONFIRMVICTIM_H_

#include "../IState.h"

/**
 * @namespace  autonohm
 */
namespace autonohm {

/**
 * @class   ConfirmVictim
 * @author  Christian Pfitzner
 * @date    2014-10-14
 */
class ConfirmVictim : public IState
{
public:
   /**
    * Default constructor
    */
   ConfirmVictim(void);
   /**
    * Default destructor
    */
   virtual ~ConfirmVictim(void);
   /**
    * Processing function
    */
   void process(void);

private:

};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_CONFIRMVICTIM_H_ */
