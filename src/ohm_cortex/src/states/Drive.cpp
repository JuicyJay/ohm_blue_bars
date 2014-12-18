/*
 * Drive.cpp
 *
 *  Created on: 18.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#include "Drive.h"

namespace autonohm {

Drive::Drive(const geometry_msgs::Pose& target)
    : _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is Drive.");
   //add subscriber:
   _subPose = _nh->subscribe("/georg/pose", 1, &Drive::subPose_callback, this);
   _ac = new actionlib::SimpleActionClient<ohm_path_plan::MoveToAction>("move_to", true);
   _pose_rdy = false;

   _targetPose.pose = target;
}

Drive::~Drive(void)
{
   delete _ac;
}

void Drive::process(void)
{

   ROS_INFO("Wait for action server...");
   _ac->waitForServer();
   ROS_INFO("Action server active...");

   while(ros::ok())
   {
      do{
         ros::spinOnce();
      }
      while(!_pose_rdy);
      //got pose:

      //transmitt to Actionserver
      ROS_INFO("Transmitt goal to server");
      ohm_path_plan::MoveToGoal goal;
      goal.start = _currentPose;
      goal.end = _targetPose;

      _ac->sendGoal(goal);

      bool succes = _ac->waitForResult(ros::Duration(200));

      if(succes)
      {
         ROS_INFO("Reached Target");
      }
      else
      {
         ROS_INFO("Timeout (100s) or no Path found");
      }

      Context::getInstance()->setState(new Explore);
      delete this;
      return;
   }
}

void Drive::subPose_callback(const geometry_msgs::PoseStamped& msg)
{
   _currentPose = msg;
   _pose_rdy = true;
}

} // end namespace autonohm
