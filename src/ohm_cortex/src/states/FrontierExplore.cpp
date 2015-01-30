/*
 * FrontierExplore.cpp
 *
 *  Created on: 30.01.2015
 *      Author: chris
 */

#include "FrontierExplore.h"

// states
#include "Drive.h"


#include "TargetStack.h"

#include "ohm_frontier_exploration/ExplorationConfig.h"




namespace autonohm {

FrontierExplore::FrontierExplore(void) :
      _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is frontier explore");

   _state_pub         = _nh->advertise<std_msgs::String>("state", 1);
   _target_srv_client = _nh->advertise<ohm_frontier_exploration::ExplorationConfig>("ohm_frontier_exploration/get_target");


   std_msgs::String msg;
   msg.data = "exploreFrontier";
   _state_pub.publish(msg);




}

FrontierExplore::~FrontierExplore() {
   // TODO Auto-generated destructor stub
}


void FrontierExplore::process(void)
{
   ohm_frontier_exploration::GetTarget service;
   geometry_msgs::Pose goal;

   if(!_target_srv_client.call(service)) {
      ROS_ERROR("Service for frontier exploration failed. ");
   }
   else {
      ROS_INFO("received new target from frontier based exploration");

      goal = service.res.target;

      ROS_INFO("Leave frontier explore state");
      Context::getInstance()->setState(new Drive(goal));
      delete this;
   }



}

} /* namespace autonohm */
