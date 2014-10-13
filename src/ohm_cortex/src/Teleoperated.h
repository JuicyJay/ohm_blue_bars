/*
 * Teleoperated.h
 *
 *  Created on: 13.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_TELEOPERATED_H_
#define OHM_CORTEX_SRC_TELEOPERATED_H_

#include "IState.h"

namespace autonohm
{

class Teleoperated : public IState
{
public:
   Teleoperated();
   virtual ~Teleoperated();
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_TELEOPERATED_H_ */
