/*
 * Inspekt.h
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_INSPEKT_H_
#define OHM_CORTEX_SRC_STATES_INSPEKT_H_

#include "../IState.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Inspekt
 * @author  Christian Pfitzner
 * @date    2014-10-14
 */
class Inspect : public IState
{
public:
   /**
    * Default constructor
    */
   Inspect(void);
   /**
    * Default destructor
    */
   virtual ~Inspect(void);
   /**
    * Function for processing
    */
   void process(void);

private:
   ros::NodeHandle*  _nh;
   ros::Publisher    _state_pub;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_INSPEKT_H_ */
