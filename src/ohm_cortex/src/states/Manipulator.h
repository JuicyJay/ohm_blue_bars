/*
 * Manipulator.h
 *
 *  Created on: 07.04.2015
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_MANIPULATOR_H_
#define OHM_CORTEX_SRC_STATES_MANIPULATOR_H_

#include "../IState.h"


#include <std_msgs/String.h>
#include "ohm_teleop/Action.h"

/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Manipulator
 * @author  Christian Pfitzner
 * @date    2015-04-07
 */
class Manipulator : public IState
{
public:
   /**
    * Default constructor
    */
   Manipulator(void);
   /**
    * Default destructor
    */
   virtual ~Manipulator(void);

   /**
    * Default processing function
    */
   void process(void);

private:
   /**
    * Callback function for state callback from inspector
    * @param state
    */
   void inspectorStateCallback(const std_msgs::String& state);
   /**
    * Callback function for joy action message
    * @param action
    */
   void actionFromJoyCallback(const ohm_teleop::Action& action);


   ros::NodeHandle*   _nh;
   ros::Publisher     _state_pub;
   ros::Subscriber    _inspector_state_sub;
   ros::Subscriber    _action_sub;


   std_msgs::String   _inspector_state;
   ohm_teleop::Action _action;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_MANIPULATOR_H_ */
