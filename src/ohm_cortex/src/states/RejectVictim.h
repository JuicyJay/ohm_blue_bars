/*
 * RejectVictim.h
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_REJECTVICTIM_H_
#define OHM_CORTEX_SRC_STATES_REJECTVICTIM_H_

#include "../IState.h"

/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   RejectVictim
 * @author  Christian Pfitzner
 * @date    2014-10-14
 */
class RejectVictim : public IState
{
public:
   /**
    * Default constructor
    */
   RejectVictim(void);
   /**
    * Default destructor
    */
   virtual ~RejectVictim(void);
   /**
    * Processing function
    */
   void process(void);

private:

};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_REJECTVICTIM_H_ */
