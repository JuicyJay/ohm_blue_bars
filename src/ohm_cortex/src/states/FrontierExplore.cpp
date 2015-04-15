/*
  * FrontierExplore.cpp
 *
 *  Created on: 30.01.2015
 *      Author: chris
 */

#include "FrontierExplore.h"

// states
#include "Drive.h"

//#include "ohm_frontier_exploration/ExplorationConfig.h"
#include "ohm_frontier_exploration/GetTarget.h"

namespace autonohm
{

FrontierExplore::FrontierExplore(void) :
      _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is frontier explore");

   _state_pub = _nh->advertise<std_msgs::String>("state", 1);
   _target_srv_client = _nh->serviceClient<ohm_frontier_exploration::GetTarget>("ohm_frontier_exploration/get_target", 1);

   std_msgs::String msg;
   msg.data = "exploreFrontier";
   _state_pub.publish(msg);

}

FrontierExplore::~FrontierExplore()
{
   // TODO Auto-generated destructor stub
}

void FrontierExplore::process(void)
{

}

} /* namespace autonohm */


