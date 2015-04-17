/*
 * FrontierExplore.h
 *
 *  Created on: 30.01.2015
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_
#define OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_

#include "../IState.h"
#include "../Context.h"

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <ohm_path_plan/PlanPath.h>

#include <ohm_srvs/NodeControl.h>
#include "../GetTransformation.h"
#include "Explore.h"

/**
 * @namespace  autonohm
 */
namespace autonohm
{

namespace frontier{
enum enum_state{
   TRIGGER_FRONTIERS = 0,
   WAIT_FRONTIERS,
   PLANNING,
   WAIT_ARIVAL
};
}

/**
 * @author Michael Schmidpeter
 *
 * @brief Singelton class for Data Stuff... e.g. Publisher and Subscriber
 */
class FrontierExploreModel
{
public:

   static FrontierExploreModel* getInstance()
   {
      if(!_instance)
         _instance = new FrontierExploreModel();
      return _instance;
   }

   ~FrontierExploreModel() { }

   bool trigger()
   {
      ohm_srvs::NodeControl srv;
      srv.request.action = (int8_t)srv.request.SINGLESHOT;

      if(_service_do_endrotate.call(srv))
      {
         if(srv.response.accepted)
         {
            return true;
         }
      }
      else
      {
         ROS_WARN("ohm_cortex: FrontierExploration -> unable to trigger frontier");
      }

      return false;
   }

   bool setEndRotate()
   {
      ohm_srvs::NodeControl srv;
      srv.request.action = (int8_t)srv.request.ENABLE;

      if(_service_do_endrotate.call(srv))
      {
         if(srv.response.accepted)
         {
            return true;
         }
      }
      else
      {
         ROS_WARN("ohm_cortex: FrontierExploration -> unable to set END_ROTATION ohm_path_control");
      }

      return false;
   }

   nav_msgs::Path requestPath(geometry_msgs::PoseStamped target)
   {
      nav_msgs::Path path;
      path.header.frame_id = "map";

      //todo use knüppl singleton class for getTransform
      //get origin
      tf::StampedTransform tf;
      try {
         ros::Time time = ros::Time::now();
         _tf_listener.waitForTransform("map", "georg/center", time, ros::Duration(5));
         _tf_listener.lookupTransform("map", "georg/center", time, tf);

      } catch (tf::TransformException& e)
      {
         ROS_ERROR("ohm_cortex: FrontierExploration -> Exeption at tf: %s", e.what());
         return path;
      }



      ohm_path_plan::PlanPath srv;
      geometry_msgs::Pose origin;
      origin.position.x = tf.getOrigin().x();
      origin.position.y = tf.getOrigin().y();
      origin.position.z = 0;

      srv.request.origin = origin;
      srv.request.target = target.pose;
      double length;
      if(_service_path.call(srv))
      {//service succes
         length = srv.response.length;
         path = srv.response.path;
         if(length < 0)
            path.poses.clear();
         return path;
      }
      else
      {
         ROS_WARN("ohm_cortex: FrontierExploration -> unable to request path");
      }
      return path;
   }

   bool isFrontierRdy() const
   {
      return _frontier_rdy;
   }

   geometry_msgs::PoseArray getFrontiers()
   {
      _frontier_rdy = false;
      return _frontiers;
   }

   bool getArivalState() const { return _state; }

   void pubTargetPath(nav_msgs::Path& path)
   {
      _pub_path.publish(path);
   }

private:
   FrontierExploreModel()
   {
      _nh = autonohm::Context::getInstance()->getNodeHandle();

      //init publisher
      _pub_path  = _nh->advertise<nav_msgs::Path>("path",1);

      //inti subscriber
      _sub_frontier = _nh->subscribe("frontiers", 1, &FrontierExploreModel::subCallback_frontiers, this);
      _sub_state    = _nh->subscribe("path_control/state", 1, &FrontierExploreModel::subCallback_state, this);

      //init service
      _service_path         = _nh->serviceClient<ohm_path_plan::PlanPath>("path_plan/srv_plan_path");
      _service_do_endrotate = _nh->serviceClient<ohm_srvs::NodeControl>("path_control/do_end_rotation");
      _service_trigger      = _nh->serviceClient<ohm_srvs::NodeControl>("frontier/node_control");
      _service_path.waitForExistence();
      _service_do_endrotate.waitForExistence();
      _service_trigger.waitForExistence();

      _state = false;
      _frontier_rdy = false;
   }
   FrontierExploreModel(FrontierExploreModel& fmodel) { }

   void subCallback_frontiers(const geometry_msgs::PoseArray& msg)
   {
      _frontier_rdy = true;
      _frontiers = msg;
   }
   void subCallback_state(const std_msgs::Bool& msg)
   {
      _state = msg.data;
   }

private:
   static FrontierExploreModel* _instance;
   //ros

   ros::NodeHandle* _nh;

   ros::Publisher _pub_path;

   ros::Subscriber _sub_frontier;
   ros::Subscriber _sub_state;

   ros::ServiceClient _service_path;
   ros::ServiceClient _service_do_endrotate;
   ros::ServiceClient _service_trigger;

   tf::TransformListener _tf_listener;

   //data
   geometry_msgs::PoseArray _frontiers;
   bool _state;
   bool _frontier_rdy;
};


/**
 * @class   FrontierExplore
 * @author  Christian Pfitzner, Michael Schmidpeter
 * @date    2015-01-30
 *
 * @brief   State class for
 */
class FrontierExplore: public IState
{
public:
   /**
    * Default constructor
    *
    * @param numFrontiers[in]  -> number of target Frontiers ... if < 0  -> endless mode (untill no frontiers)
    * @param afterState[in]    -> State called afer this , if NULL(default) then Explore will be called
    */
   FrontierExplore(int numFrontiers, IState* afterState = NULL);
   /**
    * Default destructor
    */
   virtual ~FrontierExplore(void);

   /**
    * Function for processing
    */
   void process(void);

private:
   nav_msgs::Path getBestTargetPath();
   void setAfterState();

private:
   IState* _afterState;

   ros::NodeHandle* _nh;
   ros::Publisher _state_pub;

   autonohm::FrontierExploreModel* _model;

   int _numFrontiers;
   int _cntrFrontiers;

   bool _oldArivalState;
   //bool _triggerd;

   frontier::enum_state _state;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_ */

