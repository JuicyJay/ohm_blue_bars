/*
 * Explore.h
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_EXPLORE_H_
#define OHM_CORTEX_SRC_STATES_EXPLORE_H_

#include "../IState.h"

#include <ros/ros.h>
#include <std_msgs/String.h>


/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Explore
 * @author  Christian Pfitzner
 * @date    2014-10-14
 */
class Explore: public IState
{
public:
   /**
    * Default constructor
    */
   Explore(void);
   /**
    * Default destructor
    */
   virtual ~Explore(void);
   /**
    * Function to for processing
    */
   virtual void process(void);

private:
   ros::NodeHandle*  _nh;
   ros::Publisher    _state_pub;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_EXPLORE_H_ */
