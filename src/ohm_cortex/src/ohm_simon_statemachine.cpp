/*
 * ohm_simon_statemachine.cpp
 *
 *  Created on: 07.04.2015
 *      Author: chris
 */


#include <string.h>
#include <ros/ros.h>


#include "Context.h"
#include "states/Teleoperated.h"

//#include "GetTransformation.h"

ros::ServiceServer    _state_srv;
autonohm::Context*    context;


int main(int argc, char** argv)
{
   ros::init(argc, argv, "simon_statemachine_node");
   ros::NodeHandle nh("~");

   context = autonohm::Context::getInstance();
   context->setNodeHandle(&nh);

   // Set init state
   context->setState(new autonohm::Teleoperated);

   // Init the tf listener.
  // GetTransformation::instance();

   ros::Rate r(10.0);

   while(ros::ok()) {
     context->process();
     r.sleep();
     ros::spinOnce();
   }
}




