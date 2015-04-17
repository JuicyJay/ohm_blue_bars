#include <string.h>
#include <ros/ros.h>
#include <ohm_common/RobotEvent.h>
#include <queue>

#include "Context.h"
#include "states/Approach.h"
#include "states/ConfirmVictim.h"
#include "states/Explore.h"
#include "states/Init.h"
#include "states/Inspect.h"
#include "states/RejectVictim.h"
#include "states/StatePing.h"
#include "states/StatePong.h"
#include "states/Teleoperated.h"
#include "states/Waiting.h"



#include "ohm_cortex/Force.h"
#include "GetTransformation.h"


bool force(ohm_cortex::Force::Request& req, ohm_cortex::Force::Response& res);
void subEvent_callback(const ohm_common::RobotEvent& msg);
void handleEvents();

ros::ServiceServer    _state_srv;
autonohm::Context*    context;
std::queue<ohm_common::RobotEvent> _event_queue;


int main(int argc, char** argv)
{
   ros::init(argc, argv, "statemachine_node");
   ros::NodeHandle nh("~");

   ros::Subscriber subEvent;

   context = autonohm::Context::getInstance();
   context->setNodeHandle(&nh);

   _state_srv = nh.advertiseService("force_state", force);
   subEvent   = nh.subscribe("/georg/event",20, subEvent_callback);

   // Set init state
   context->setState(new autonohm::Init);

   // Init the tf listener.
   GetTransformation::instance();

   ros::Rate r(10.0);

   while(ros::ok())
   {
     context->process();
     r.sleep();
     ros::spinOnce();
     handleEvents();
   }
}



bool force(ohm_cortex::Force::Request& req, ohm_cortex::Force::Response& res)
{

   std::cout << __PRETTY_FUNCTION__ << " : " << req.state << std::endl;

   if(req.state == "approach" )      context->setState(new autonohm::Approach());
   if(req.state == "init"     )      context->setState(new autonohm::ConfirmVictim());
   if(req.state == "explore"  )      context->setState(new autonohm::Explore());
   if(req.state == "confirmvictim")  context->setState(new autonohm::Init());
//   if(req.state == "inspect")        context->setState(new autonohm::Inspect());
   if(req.state == "rejectvictim")   context->setState(new autonohm::RejectVictim());
   if(req.state == "stateping")      context->setState(new autonohm::StatePing());
   if(req.state == "statepong")      context->setState(new autonohm::StatePong());
   if(req.state == "teleoperated")   context->setState(new autonohm::Teleoperated());
   if(req.state == "waiting"  )      context->setState(new autonohm::Waiting());


   return true;
}

void subEvent_callback(const ohm_common::RobotEvent& msg)
{
   _event_queue.push(msg);
}

void handleEvents()
{
   while(_event_queue.size())
   {
      ohm_common::RobotEvent msg = _event_queue.front();

      if(msg.event == ohm_common::RobotEvent::FLIP_OVER)
      {
         ROS_INFO("ohm_cortex -> EVENT -> FLIP_OVER");
      }
      else if(msg.event == ohm_common::RobotEvent::MOVING_ABORTED)
      {
         ROS_INFO("ohm_cortex -> EVENT -> MOVING_ABORTED");
      }
      else if(msg.event == ohm_common::RobotEvent::MOVING_PAUSED)
      {
         ROS_INFO("ohm_cortex -> EVENT -> MOVING_PAUSED");
      }
      else
      {
         ROS_INFO("ohm_cortex -> EVENT -> Undefined");
      }
      _event_queue.pop();
   }
}
