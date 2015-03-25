/*
 * Drive.cpp
 *
 *  Created on: 18.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#include "Drive.h"
#include "Explore.h"
#include "LookAtTarget.h"

namespace autonohm {

ros::Publisher* Drive::_pubPath = NULL;
ros::Publisher* Drive::_pubTarget = NULL;

Drive::Drive(const geometry_msgs::Pose& target)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _stateAfter(0)
{
   ROS_INFO("New state is Drive -> default... just drive.");

   /* Publish current state for debugging. */
   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   _state = drive::WAIT_FOR_SUB;
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

Drive::Drive(const geometry_msgs::Point& target, const geometry_msgs::Quaternion& orientation, IState* stateAfter)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _stateAfter(stateAfter)
{
   ROS_INFO("New state is Drive -> mode: no rotate, Inspect");

   /* Publish current state for debugging. */
   _state_pub = _nh->advertise<std_msgs::String>("state", 1);

   _state = drive::WAIT_FOR_SUB;
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
   switch (_state) {

      case drive::WAIT_FOR_SUB:
         ROS_INFO("ohm_cortex: Drive -> Prove to Subscripber");
         if(_pubTarget->getNumSubscribers() == 0 || _pubPath->getNumSubscribers() == 0)
         {
            ROS_INFO("ohm_cortex: Drive -> -- NO Subscriber");
            return;
         }

         _state = drive::PUB_TAEGET;
         break;
      case drive::PUB_TAEGET:
         //publish target
         _pubTarget->publish(_targetPose);

         _state = drive::WAIT_FOR_PATH;
         break;
      case drive::WAIT_FOR_PATH:

         //wait for path:
         if(!_got_path)
            return;

         ROS_INFO("ohm_cortex: Drive -> Got Path: %d",(int)_path.poses.size());
         _state = drive::PUB_PATH;
         break;
      case drive::PUB_PATH:
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
         }
         _pubPath->publish(_path);

         ROS_INFO("ohm_cortex: Drive -> Wait for arival");
         _state = drive::WAIT_FOR_ARIVAL;
         break;
      case drive::WAIT_FOR_ARIVAL:
         if(!_reached_target)
            return;
         ROS_INFO("ohm_cortex: Drive ->  Arrived!!!!");

         _state = drive::DESTROY;
         break;
      case drive::DESTROY:
         if(_mode == drive::NO_TARGET_ORI)
         {//next inspect
            ROS_INFO("ohm_cortex: Drive -> Call Inspect");
            Context::getInstance()->setState(_stateAfter ? _stateAfter : new LookAtTarget(_targetOrientation));
         }
         else
         {//next explore
            ROS_INFO("ohm_cortex: Drive -> Call Explore");
            Context::getInstance()->setState(new Explore);
         }
         _state = drive::NONE;
         delete this;
         break;
      default:
         return;
         break;

   }

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
