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

namespace{
const double MAP_CELL_SIZE = 0.015;
const double TRANCATE_LENGTH = 0.75;
}

autonohm::FrontierExploreModel* autonohm::FrontierExploreModel::_instance = NULL;

namespace autonohm
{

FrontierExplore::FrontierExplore(int numFrontiers, IState* afterState) :
      _nh(autonohm::Context::getInstance()->getNodeHandle())
{
   ROS_INFO("New state is frontier explore");

   _afterState = afterState;

   _state_pub = _nh->advertise<std_msgs::String>("state", 1);
   _sub_map   = _nh->subscribe("georg/map", 1, &FrontierExplore::subCallback_map, this);


   std_msgs::String msg;
   msg.data = "exploreFrontier";
   _state_pub.publish(msg);

   _model = autonohm::FrontierExploreModel::getInstance();

   _numFrontiers = numFrontiers;
   _cntrFrontiers = 0;

   _state = frontier::TRIGGER_FRONTIERS;
   _oldArivalState = false;
   _moving = false;
   //_triggerd = false;
}


FrontierExplore::~FrontierExplore()
{

}

void FrontierExplore::process(void)
{
   switch (_state) {

      case frontier::TRIGGER_FRONTIERS:
         ROS_INFO("FrontierState -> Trigger Frontiers");
         if(_model->trigger())
            _state = frontier::WAIT_FRONTIERS;
         break;

      case frontier::WAIT_FRONTIERS:
         //ROS_INFO("FrontierState -> Wait_Frontiers");
         if(_model->isFrontierRdy())
            _state = frontier::PLANNING;
         break;

      case frontier::PLANNING:
      {
         ROS_INFO("FrontierState -> Planning");
         nav_msgs::Path path_;
         path_= this->getBestTargetPath();

         if(!path_.poses.size())
         {
            ROS_INFO("ohm_cortex: FrontierExploraion -> no acceptable frontiers!!! no moving possible");
            this->setAfterState();
         }
         else
         {
            //transmitt path
            ROS_INFO("FrontierState -> Pub path");
            path_.header.frame_id = "map";
            _model->pubTargetPath(path_);
            _moving = true;
            _state = frontier::WAIT_ARIVAL;
         }

         break;
      }
      case frontier::WAIT_ARIVAL:
         //ROS_INFO("FrontierState -> Wait for arival");
         if(!_oldArivalState && _model->getArivalState())
         {//arrived
            this->callArrived();

         }
         _oldArivalState = _model->getArivalState();

         break;
      default:
         break;
   }

}

nav_msgs::Path FrontierExplore::getBestTargetPath()
{
   nav_msgs::Path path;

   geometry_msgs::PoseArray frontiers = _model->getFrontiers();

   for(unsigned int i = 0; i < frontiers.poses.size(); ++i)
   {
      geometry_msgs::PoseStamped targetPose;
      targetPose.pose = frontiers.poses[i];
      path = _model->requestPath(targetPose);
      if(path.poses.size())
      {//found path
         //truncate path
         unsigned int num_cut = (TRANCATE_LENGTH / MAP_CELL_SIZE) + 0.555;

         if(path.poses.size() > num_cut)
         {
            nav_msgs::Path path_trunc;

            for(unsigned int k = 0; k < path.poses.size() - num_cut; ++k)
            {
               path_trunc.poses.push_back(path.poses[k]);
            }
            path_trunc.poses.back().pose.orientation = frontiers.poses[i].orientation;
            _targetPose = targetPose;
            return path_trunc;
         }
         else
         {//to short path... just rotate to target pose
            nav_msgs::Path tmp_path;
            tmp_path.poses.push_back(path.poses[0]);
            //set orientation
            tmp_path.poses[0].pose.orientation = frontiers.poses[i].orientation;
            _targetPose = targetPose;
            return tmp_path;
         }

         break;
      }
   }

   return path;
}

void FrontierExplore::setAfterState()
{
   if(_afterState)
   {
      Context::getInstance()->setState(_afterState);
      delete this;
   }
   else
   {
      Context::getInstance()->setState(new Explore);
      delete this;
   }
}

void FrontierExplore::subCallback_map(const nav_msgs::OccupancyGrid& msg)
{
   if(_moving)
   {//plan new path

      if(!this->pubPath(_model->requestPath(_targetPose)))
      {//aboard moving to target... set arrived.
         //stop
         nav_msgs::Path empty_path;
         empty_path.header.frame_id = "map";
         empty_path.poses.clear();
         _model->pubTargetPath(empty_path);

         this->callArrived();
      }
   }
}

bool FrontierExplore::pubPath(nav_msgs::Path path)
{
   if(!path.poses.size())
   {//empty path
      path.header.frame_id = "map";
      _model->pubTargetPath(path);
      return false;
   }

   geometry_msgs::PoseStamped target;
   //truncate path
   unsigned int num_cut = (TRANCATE_LENGTH / MAP_CELL_SIZE) + 0.555;

   if(path.poses.size() > num_cut)
   {
      nav_msgs::Path path_trunc;

      for(unsigned int k = 0; k < path.poses.size() - num_cut; ++k)
      {
         path_trunc.poses.push_back(path.poses[k]);
      }
      path_trunc.poses.back().pose.orientation = target.pose.orientation;
      //_targetPose = targetPose;
      path_trunc.header.frame_id = "map";
      _model->pubTargetPath(path_trunc);
      return true;
   }
   else
   {//to short path... just rotate to target pose
      nav_msgs::Path tmp_path;
      tmp_path.poses.push_back(path.poses[0]);
      //set orientation
      tmp_path.poses[0].pose.orientation = target.pose.orientation;
      //_targetPose = targetPose;
      tmp_path.header.frame_id = "map";
      _model->pubTargetPath(tmp_path);
      return true;
   }
}


void FrontierExplore::callArrived()
{
   _moving = false;
   if(_numFrontiers < 0)
   {//endless mode... untill no frontiers availible
      _state = frontier::TRIGGER_FRONTIERS;
   }
   else if(_cntrFrontiers >= (_numFrontiers - 1))
   {//end state
      this->setAfterState();
   }
   else
   {
      _cntrFrontiers++;
      _state = frontier::TRIGGER_FRONTIERS;
   }
}

} /* namespace autonohm */


