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

   std_msgs::String msg;
   msg.data = "exploreFrontier";
   _state_pub.publish(msg);

   _model = autonohm::FrontierExploreModel::getInstance();

   _numFrontiers = numFrontiers;
   _cntrFrontiers = 0;

   _state = frontier::WAIT_FRONTIERS;
   _oldArivalState = false;
   //_triggerd = false;
}


FrontierExplore::~FrontierExplore()
{

}

void FrontierExplore::process(void)
{
   switch (_state) {

      case frontier::TRIGGER_FRONTIERS:
         if(_model->trigger())
            _state = frontier::WAIT_FRONTIERS;
         break;

      case frontier::WAIT_FRONTIERS:
         if(_model->isFrontierRdy())
            _state = frontier::PLANNING;
         break;

      case frontier::PLANNING:
      {
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
            _model->pubTargetPath(path_);
            _state = frontier::WAIT_ARIVAL;
         }

         break;
      }
      case frontier::WAIT_ARIVAL:
         if(!_oldArivalState && _model->getArivalState())
         {//arrived
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
            return path_trunc;
         }
         else
         {//to short path... just rotate to target pose
            nav_msgs::Path tmp_path;
            tmp_path.poses.push_back(path.poses[0]);
            //set orientation
            tmp_path.poses[0].pose.orientation = frontiers.poses[i].orientation;
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

} /* namespace autonohm */


