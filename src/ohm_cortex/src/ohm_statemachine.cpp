#include <ros/ros.h>

#include "Context.h"
#include "StatePing.h"

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ohm_statemachine_node");
   ros::NodeHandle nh("~");

   autonohm::Context* context = autonohm::Context::getInstance();

   // Set init state
   context->setState(new autonohm::StatePing());

   while(ros::ok())
   {
     context->process();

     ros::spinOnce();
   }
}
