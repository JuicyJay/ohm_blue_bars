/*
 * Teleoperated.h
 *
 *  Created on: 13.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_TELEOPERATED_H_
#define OHM_CORTEX_SRC_TELEOPERATED_H_


// ohm_cortex include
#include "../IState.h"

/**
 * @namespace autonohm
 */
namespace autonohm
{

/**
 * @class   Teleoperated
 * @author  Christian Pfitzner
 * @date    2014-10-14
 *
 * @brief
 */
class Teleoperated : public IState
{
public:
   /**
    * Default constructor
    */
   Teleoperated(void);
   /**
    * Default destructor
    */
   virtual ~Teleoperated(void);
   /**
    * Function for processing while in state
    */
   void process(void);
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_TELEOPERATED_H_ */
