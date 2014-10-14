#include <ros/ros.h>

#include "Context.h"
#include "states/Init.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ohm_statemachine_node");
   ros::NodeHandle nh("~");

   autonohm::Context* context = autonohm::Context::getInstance();
   context->setNodeHandle(&nh);

   // Set init state
   context->setState(new autonohm::Init());

   ros::Rate r(10.0);

   while(ros::ok())
   {
     context->process();
     r.sleep();
     ros::spinOnce();
   }
}
