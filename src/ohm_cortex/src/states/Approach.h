/*
 * Approach.h
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_APPROACH_H_
#define OHM_CORTEX_SRC_STATES_APPROACH_H_

#include "../IState.h"

/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Approach
 * @author  Christian Pfitzner
 * @date    2014-10-14
 */
class Approach: public IState
{
public:
   /**
    * Default constructor
    */
   Approach(void);
   /**
    * Default destructor
    */
   virtual ~Approach();
   /**
    * Function for processing
    */
   void process(void);

private:
   ros::NodeHandle*  _nh;

   ros::Publisher    _state_pub;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_APPROACH_H_ */
