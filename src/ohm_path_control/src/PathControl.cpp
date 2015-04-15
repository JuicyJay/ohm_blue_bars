
#include "PathControl.h"
#include "PathAnalyser/SimpleAnalyser.h"
#include "PathAnalyser/BasicAnalyser.h"
#include "Controller/ParabolaTransfere.h"

PathControl::PathControl() : _rate(0)
{
    _loopRate = 0;

    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_name_cmd_vel;
    std::string pub_name_state;
    std::string pub_name_state_full;
    std::string sub_name_path;
    std::string sub_name_em_stop;
    std::string sub_name_pause;
    std::string srv_name_ctrl_endroate;

    std::string config_file_controller;
    std::string config_file_analyser;
    std::string tf_map_frame;
    std::string tf_robot_frame;
    
    privNh.param("pub_name_cmd_vel",       pub_name_cmd_vel,       std::string("vel/teleop"));
    privNh.param("pub_name_state",         pub_name_state,         std::string("path_control/state"));
    privNh.param("pub_name_state_full",    pub_name_state_full,    std::string("path_control/state_full"));
    privNh.param("sub_name_path",          sub_name_path,          std::string("path"));
    privNh.param("sub_name_em_stop",       sub_name_em_stop,       std::string("path_control/emergency_stop"));
    privNh.param("sub_name_pause",         sub_name_pause,         std::string("path_control/pause"));
    privNh.param("srv_name_ctrl_endroate", srv_name_ctrl_endroate, std::string("path_control/ctrl_endrotate"));
    privNh.param("config_file_controller", config_file_controller, std::string("/home/m1ch1/workspace/ros/ohm_autonomy/src/ohm_path_control/config/controller.xml"));
    privNh.param("config_file_analyser",   config_file_analyser,   std::string("/home/m1ch1/workspace/ros/ohm_autonomy/src/ohm_path_control/config/analyser.xml"));
    privNh.param("tf_map_frame",           tf_map_frame,           std::string("map"));
    privNh.param("tf_robot_frame",         tf_robot_frame,         std::string("base_footprint"));

    _tf_map_frame = tf_map_frame;
    _tf_robot_frame = tf_robot_frame;

    //init publisher
    _pub_cmd_vel = _nh.advertise<geometry_msgs::Twist>(pub_name_cmd_vel,1);
    _pub_state = _nh.advertise<ohm_autonomy::PathControlInfo>(pub_name_state_full,1);
    _pub_state_old = _nh.advertise<std_msgs::Bool>(pub_name_state,1);

    //inti subscriber
    _sub_path = _nh.subscribe(sub_name_path , 1, &PathControl::subPath_callback, this);
    _sub_em_stop = _nh.subscribe(sub_name_em_stop, 1, &PathControl::subEmStop_callback, this);
    _sub_pause = _nh.subscribe(sub_name_pause, 1, &PathControl::subPause_callback, this);

    //init srv
    _srv_nodeControl_endrotate = _nh.advertiseService(srv_name_ctrl_endroate, &PathControl::srvCntrlEndrotate_callback, this);

    //_pathAnalyser = new analyser::SimpleAnalyser(config_file_analyser);
    _pathAnalyser = new analyser::BasicAnalyser(config_file_analyser);
    _controller = new controller::ParabolaTransfere(config_file_controller);

    _enable_analyse = false;
    _pause = false;
    _em_stop = false;
}

PathControl::~PathControl()
{
    delete _rate;
}

void PathControl::start(const unsigned int rate)
{
    delete _rate;
    _loopRate = rate;
    _rate = new ros::Rate(_loopRate);

    //wait for first transform
    bool rdy = false;
    do{
       try {
          ros::Time time = ros::Time::now();
          _tf_listnener.waitForTransform(_tf_map_frame, _tf_robot_frame, time, ros::Duration(1));
          rdy = true;

       } catch (tf::TransformException& e)
       {
          ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
          return;
       }
    }while(!rdy);
   
    this->run();
}

void PathControl::run()
{
    unsigned int cnt = 0;

    while(ros::ok())
    {
       this->doPathControl();
       
       ros::spinOnce();
       _rate->sleep();
    }
}

void PathControl::doPathControl(void)
{
   if(_em_stop)
   {
      ROS_WARN("ohm_path_control -> EMERNGENCY_STOP activated!!!! moving canceld");

      geometry_msgs::Twist msgTwist;
      msgTwist.angular.z = 0;
      msgTwist.linear.x = 0;

      _pub_cmd_vel.publish(msgTwist);

      //delete path:
      std::vector<analyser::pose> path;
      //set empty path
      _pathAnalyser->setPath(path);

      return;
   }
   if(!_enable_analyse)
   {
      return;
   }

   //get tf
   tf::StampedTransform tf;
   try {
      ros::Time time = ros::Time(0);
      _tf_listnener.lookupTransform(_tf_map_frame, _tf_robot_frame, time, tf);

   } catch (tf::TransformException& e)
   {
      ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
      return;
   }

   analyser::pose pose;
   pose.position = Vector3d( tf.getOrigin().x(), tf.getOrigin().y(), 0);
   Quaternion<double> tmp_q(tf.getRotation().w(),
                            tf.getRotation().x(),
                            tf.getRotation().y(),
                            tf.getRotation().z() );
   pose.orientation = analyser::PathAnalyser_base::quaternion_to_orientationVec(tmp_q);

   //get diff scale
   analyser::diff_scale diff_scale = _pathAnalyser->analyse(pose);

   //controll diffscale
   controller::velocity vel = _controller->control(diff_scale.linear, diff_scale.angular);

   //set twist msg
   geometry_msgs::Twist msgTwist;
   msgTwist.angular.z = vel.angular;
   msgTwist.linear.x = vel.linear;

//   if(pathInfo.reached_final_goal)
//   {
//      //_enable_analyse = false;
//   }

   if(_pause)
   {
      msgTwist.angular.z = 0;
      msgTwist.linear.x = 0;
   }


   //for old msg
   std_msgs::Bool state_msg_old;
   state_msg_old.data = false;

   if(_pathAnalyser->isReachedFinalGoal())
      state_msg_old.data = true;

   _pub_state_old.publish(state_msg_old);

   this->pubState();

   //publish Twist:
   _pub_cmd_vel.publish(msgTwist);
}


void PathControl::pubState(void)
{
   analyser::info pathInfo = _pathAnalyser->getInfo();
   ohm_autonomy::PathControlInfo msg_state;
   msg_state.pathLenght = pathInfo.path_length;
   msg_state.pathLenght_remaining = pathInfo.path_length_remaining;
   msg_state.numWaypoints = pathInfo.num_goals;
   msg_state.currentWaypoint = pathInfo.current_goal_id;
   msg_state.doEndRotate = _pathAnalyser->isDoEndRotate();


   if(pathInfo.reached_goal)
   {
      msg_state.state = msg_state.ARRIVED;
      msg_state.state_str = "ARRIVED";
   }
   else
   {
      msg_state.state = msg_state.MOVING;
      msg_state.state_str = "MOVING";
   }

   if(_pause)
   {
      msg_state.state = msg_state.PAUSE;
      msg_state.state_str = "PAUSE";
   }

   if(_em_stop)
   {
      msg_state.state = msg_state.STOP;
      msg_state.state_str = "STOP";
   }

   _pub_state.publish(msg_state);
}

void PathControl::subPath_callback(const nav_msgs::Path& msg)
{
   std::vector<analyser::pose> path_comp;
   std::vector<analyser::pose> path_trunc;

   path_comp.resize(msg.poses.size());
   path_trunc.resize(msg.poses.size());

   ROS_INFO("ohm_path_control -> GOT Path");

   //set path
   for(unsigned int i=0; i<msg.poses.size(); i++)
   {
      analyser::pose tmp_pose;
      tmp_pose.position = Vector3d(msg.poses[i].pose.position.x,
                                   msg.poses[i].pose.position.y,
                                   msg.poses[i].pose.position.z);

      Quaternion<double> tmp_q(msg.poses[i].pose.orientation.w,
                               msg.poses[i].pose.orientation.x,
                               msg.poses[i].pose.orientation.y,
                               msg.poses[i].pose.orientation.z);

      tmp_pose.orientation = analyser::PathAnalyser_base::quaternion_to_orientationVec(tmp_q);
      path_comp[i] = tmp_pose;
      path_trunc[i] = tmp_pose;
   }

   //get tf
   bool tf_rdy = true;
   tf::StampedTransform tf;
   do{
      try {
         ros::Time time = ros::Time(0);
         _tf_listnener.lookupTransform(_tf_map_frame, _tf_robot_frame, time, tf);

      } catch (tf::TransformException& e)
      {
         ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
         tf_rdy = false;
      }
   }while(!tf_rdy);
   //tf rdy

   //now porve if path is in radius

   analyser::BasicAnalyser* tmp_analyser = dynamic_cast<analyser::BasicAnalyser*>(_pathAnalyser);
   if(tmp_analyser)
   {//just do if cast is valid
      double detectionRaidus = tmp_analyser->getDetectionRadius();

      detectionRaidus *= 0.5; // todo use parameter to config this

      //px, py : robot pose
      double px = tf.getOrigin().x();
      double py = tf.getOrigin().y();

      unsigned int trunc_idx = 0;
      for(int i = (int)path_comp.size() - 1; i >= 0; --i)
      {
         //cx, cy : current path point
         double cx = path_comp[i].position.x();
         double cy = path_comp[i].position.y();

         //prove if path point is in robot pose + detection radius: (x-x0)^2 + (y-y0)^2 < r^2
         if( (cx - px)*(cx - px) + (cy - py)*(cy - py) < (detectionRaidus * detectionRaidus) )
         {//point is in radius
            ROS_INFO("Found stuff in Radius!!!!!!!!!!!!!");
            trunc_idx = i;
            break;
         }
      }

      if(trunc_idx)
         path_trunc.erase(path_trunc.begin(), path_trunc.begin() + trunc_idx);

      if(!path_trunc.size())
      {//if path_tranc contains no points than use path_comp;
         path_trunc = path_comp;
      }
   }
   else
   {
      path_trunc = path_comp;
   }



   //prove last path element of nan
   if(msg.poses.size())
   {
      if(isnan(msg.poses[msg.poses.size() - 1].pose.orientation.w) ||
         isnan(msg.poses[msg.poses.size() - 1].pose.orientation.x) ||
         isnan(msg.poses[msg.poses.size() - 1].pose.orientation.y) ||
         isnan(msg.poses[msg.poses.size() - 1].pose.orientation.z)   )
      {
         //if none is nan ... than set to norotate
         ROS_INFO("ohm_path_control -> Disable Rotate");
         _pathAnalyser->setDoEndRotate(false);
      }
      else
      {
         ROS_INFO("ohm_path_control -> Enable Rotate");
         _pathAnalyser->setDoEndRotate(true);
      }
   }

   ROS_INFO("Path.size: %d",(int)path_trunc.size());
   _pathAnalyser->setPath(path_trunc);
   _enable_analyse = true;
   std_msgs::Bool reachedTarget;
   reachedTarget.data = false;
   for(unsigned int i = 0; i < 50; ++i)
   {
      this->pubState();
      _pub_state_old.publish(reachedTarget);
      usleep(10000);
   }
}

void PathControl::subEmStop_callback(const std_msgs::Bool& msg)
{
   _em_stop = msg.data;
}

void PathControl::subPause_callback(const std_msgs::Bool& msg)
{
   _pause = msg.data;
}



bool PathControl::srvCntrlEndrotate_callback(ohm_srvs::NodeControlRequest& req,
      ohm_srvs::NodeControlResponse& res)
{
   res.accepted = true;

   if(req.action == req.ENABLE)
   {
      _pathAnalyser->setDoEndRotate(true);
   }
   else if(req.action == req.DISABLE)
   {
      _pathAnalyser->setDoEndRotate(false);
   }
   else
   {
      res.accepted = false;
   }
   return true;
}
