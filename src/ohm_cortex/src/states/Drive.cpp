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

   /* Publish current state for debugging. */
   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   std_msgs::String msg;
   msg.data = "drive";
   _state_pub.publish(msg);

   //add publisher
   _pubTarget = _nh->advertise<geometry_msgs::PoseStamped>("/georg/target", 1);
   _pubPath   = _nh->advertise<nav_msgs::Path>("/georg/path", 1);

   //add subscriber:
   _subPath  = _nh->subscribe("/georg/target_path", 1, &Drive::subPath_callback, this);
   _subState = _nh->subscribe("/georg/path_control/state", 1, &Drive::subState_callback, this);
   _old_state = true;
   _reached_target = false;
   _got_path = false;

   _targetPose.pose = target;
   std::cout << "target = (" << target.position.x << ", " << target.position.y << ")" << std::endl;
}

Drive::~Drive(void)
{
}

void Drive::process(void)
{
   ROS_INFO("ohm_cortex: Drive -> Prove to Subscripber");
   if(_pubTarget.getNumSubscribers() == 0 || _pubPath.getNumSubscribers() == 0)
   {
      ROS_INFO("ohm_cortex: Drive -> -- NO Subscriber");
      return;
   }

   //publish target
   _pubTarget.publish(_targetPose);

   //wait for path:
   do{
      ros::spinOnce();
      usleep(5000); // sleep 5 ms
   }
   while(!_got_path);
   ROS_INFO("ohm_cortex: Drive -> Got Path");

   //prove path
   if(_path.poses.size() == 0)
   {
      ROS_INFO("No Path found ... return to ExloreState");
      Context::getInstance()->setState(new Explore);
      delete this;
      return;
   }

   ROS_INFO("ohm_cortex: Drive -> Transmitt Path");
   //got path:
   //now pubish to pathcontroll and begin moving until target reached
   _pubPath.publish(_path);

   ROS_INFO("ohm_cortex: Drive -> Wait for arival");
   do{
      ros::spinOnce();
      usleep(5000); // sleep 5 ms
   }
   while(!_reached_target);

   ROS_INFO("ohm_cortex: Drive ->  Arrived!!!!");

   Context::getInstance()->setState(new Explore);
   delete this;
   return;
}

void Drive::subPath_callback(const nav_msgs::Path& msg)
{
   _path = msg;
   _got_path = true;
}

void Drive::subState_callback(const std_msgs::Bool& msg)
{
   bool tmp = msg.data;
   //get rising edge
   if(!_old_state && tmp)
   {
      _reached_target = true;
   }
   _old_state = tmp;
}

} // end namespace autonohm
