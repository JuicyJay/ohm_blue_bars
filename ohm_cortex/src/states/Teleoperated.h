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

#include <std_msgs/String.h>


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

private:
   /**
    * Callback function for state of inspector
    * @param state
    */
   void inspectorStateCallback(const std_msgs::String& state);


   ros::NodeHandle*   _nh;
   ros::Publisher     _state_pub;
   ros::Subscriber    _inspector_state_sub;

   std_msgs::String   _inspector_state;

};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_TELEOPERATED_H_ */
