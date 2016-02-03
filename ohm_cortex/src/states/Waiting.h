/*
 * Wainting.h
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_WAINTING_H_
#define OHM_CORTEX_SRC_STATES_WAINTING_H_

#include <ros/ros.h>

#include "../IState.h"


/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Waiting
 * @author  Christian Pfitzner
 * @date    2014-10-14
 */
class Waiting : public IState
{
public:
   /**
    * Default constructor

    */
   Waiting(void);
   /**
    * Default destructor
    */
   virtual ~Waiting(void);
   /**
    * Function for processing
    */
   void process(void);

private:

   ros::NodeHandle* _nh;
   ros::Publisher   _state_pub;
   ros::ServiceClient _srvControlMap;

};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_WAINTING_H_ */
