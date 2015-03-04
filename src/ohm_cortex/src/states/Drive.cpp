/*
 * Drive.cpp
 *
 *  Created on: 18.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#include "Drive.h"

namespace autonohm {

ros::Publisher* Drive::_pubPath = NULL;
ros::Publisher* Drive::_pubTarget = NULL;

Drive::Drive(const geometry_msgs::Pose& target)
    : _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is Drive -> default... just drive.");

   /* Publish current state for debugging. */
   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   _mode = drive::DEFAULT;

   std_msgs::String msg;
   msg.data = "drive";
   _state_pub.publish(msg);

   //add publisher
   if(_pubTarget == NULL)
   {
      _pubTarget = new ros::Publisher;
      *_pubTarget = _nh->advertise<geometry_msgs::PoseStamped>("/georg/target", 1);
   }
   if(_pubPath == NULL)
   {
      _pubPath = new ros::Publisher;
      *_pubPath   = _nh->advertise<nav_msgs::Path>("/georg/path", 1);
   }

   //add subscriber:
   _subPath  = _nh->subscribe("/georg/target_path", 1, &Drive::subPath_callback, this);
   _subState = _nh->subscribe("/georg/path_control/state", 1, &Drive::subState_callback, this);
   _old_state = true;
   _reached_target = false;
   _got_path = false;

   _targetPose.header.frame_id = "map";
   _targetPose.pose = target;
   std::cout << "target = (" << target.position.x << ", " << target.position.y << ")" << std::endl;
}

Drive::Drive(const geometry_msgs::Point& target, geometry_msgs::Quaternion& orientation)
      : _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is Drive -> mode: no rotate, Inspect");

   /* Publish current state for debugging. */
   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   _mode = drive::NO_TARGET_ORI;
   _targetOrientation = orientation;

   std_msgs::String msg;
   msg.data = "drive";
   _state_pub.publish(msg);

   //add publisher
   //add publisher
   if(_pubTarget == NULL)
   {
      _pubTarget = new ros::Publisher;
      *_pubTarget = _nh->advertise<geometry_msgs::PoseStamped>("/georg/target", 1);
   }
   if(_pubPath == NULL)
   {
      _pubPath = new ros::Publisher;
      *_pubPath   = _nh->advertise<nav_msgs::Path>("/georg/path", 1);
   }

   //add subscriber:
   _subPath  = _nh->subscribe("/georg/target_path", 1, &Drive::subPath_callback, this);
   _subState = _nh->subscribe("/georg/path_control/state", 1, &Drive::subState_callback, this);
   _old_state = true;
   _reached_target = false;
   _got_path = false;

   _targetPose.header.frame_id = "map";
   _targetPose.pose.position = target;
   _targetPose.pose.orientation = orientation;
   //std::cout << "target = (" << target.x << ", " << target.y << ")" << std::endl;
}

Drive::~Drive(void)
{
}

void Drive::process(void)
{
   ROS_INFO("ohm_cortex: Drive -> Prove to Subscripber");
   if(_pubTarget->getNumSubscribers() == 0 || _pubPath->getNumSubscribers() == 0)
   {
      ROS_INFO("ohm_cortex: Drive -> -- NO Subscriber");
      return;
   }

   //publish target
   _pubTarget->publish(_targetPose);

   //wait for path:
   do{
      ros::spinOnce();
      usleep(5000); // sleep 5 ms
   }
   while(!_got_path);
   ROS_INFO("ohm_cortex: Drive -> Got Path: %d",(int)_path.poses.size());

   //prove path
   if(_path.poses.size() == 0)
   {
      ROS_INFO("ohm_cortex: Drive -> No Path found ... return to ExloreState");
      Context::getInstance()->setState(new Explore);
      delete this;
      return;
   }

   ROS_INFO("ohm_cortex: Drive -> Transmitt Path");
   //got path:
   //now pubish to pathcontroll and begin moving until target reached
   if(_mode == drive::NO_TARGET_ORI)
   {//change last wp orientation

      geometry_msgs::Quaternion ori;
      ori.x = std::numeric_limits<double>::quiet_NaN();
      ori.y = std::numeric_limits<double>::quiet_NaN();
      ori.z = std::numeric_limits<double>::quiet_NaN();
      ori.w = std::numeric_limits<double>::quiet_NaN();

      _path.poses[_path.poses.size() - 1].pose.orientation = ori;
//
//      if(_path.poses.size() >= 2)
//      {
//         geometry_msgs::Point p_last = _path.poses[_path.poses.size() - 1].pose.position;
//         geometry_msgs::Point p_blast = _path.poses[_path.poses.size() - 2].pose.position;
//         Eigen::Vector3d l(p_last.x, p_last.y, p_last.z);   //last
//         Eigen::Vector3d bl(p_blast.x, p_blast.y, p_blast.z);  //before last
//
//         Eigen::Quaternion<double> q;
//         q = q.FromTwoVectors(Eigen::Vector3d(1,0,0), l-bl);
//
//         geometry_msgs::Quaternion ori;
//         ori.x = q.x();
//         ori.y = q.y();
//         ori.z = q.z();
//         ori.w = q.w();
//         _path.poses[_path.poses.size() - 1].pose.orientation = ori;
//      }
   }
   _pubPath->publish(_path);

   ROS_INFO("ohm_cortex: Drive -> Wait for arival");
   do{
      ros::spinOnce();
      usleep(5000); // sleep 5 ms
   }
   while(!_reached_target);

   ROS_INFO("ohm_cortex: Drive ->  Arrived!!!!");

   if(_mode == drive::NO_TARGET_ORI)
   {//next inspect
      ROS_INFO("ohm_cortex: Drive -> Call Inspect");
      Context::getInstance()->setState(new Inspect(_targetOrientation));
   }
   else
   {//next explore
      ROS_INFO("ohm_cortex: Drive -> Call Explore");
      Context::getInstance()->setState(new Explore);
   }
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
      ROS_INFO("ohm_cortex: Drive -> #################################################Reached target");
      _reached_target = true;
   }
   _old_state = tmp;
}

} // end namespace autonohm
