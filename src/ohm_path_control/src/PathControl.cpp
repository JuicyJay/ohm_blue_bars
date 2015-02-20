
#include "PathControl.h"
#include "PathAnalyser/SimpleAnalyser.h"
#include "Controller/ParabolaTransfere.h"

PathControl::PathControl() : _rate(0)
{
    _loopRate = 0;

    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_name_cmd_vel;
    std::string pub_name_state;
    std::string sub_name_path;
    std::string sub_name_em_stop;
    std::string sub_name_pause;
    //std::string sub_name_pose;
    std::string config_file_controller;
    std::string config_file_analyser;
    std::string tf_map_frame;
    std::string tf_robot_frame;
    
    privNh.param("pub_name_cmd_vel",       pub_name_cmd_vel,       std::string("vel/teleop"));
    privNh.param("pub_name_state",         pub_name_state,         std::string("path_control/state"));
    privNh.param("sub_name_path",          sub_name_path,          std::string("path"));
    privNh.param("sub_name_em_stop",       sub_name_em_stop,       std::string("path_control/emergency_stop"));
    privNh.param("sub_name_pause",         sub_name_pause,         std::string("path_control/pause"));
    //privNh.param("sub_name_pose",          sub_name_pose,          std::string("pose"));
    privNh.param("config_file_controller", config_file_controller, std::string("/home/m1ch1/workspace/ros/ohm_autonomy/src/ohm_path_control/config/controller.xml"));
    privNh.param("config_file_analyser",   config_file_analyser,   std::string("/home/m1ch1/workspace/ros/ohm_autonomy/src/ohm_path_control/config/analyser.xml"));
    privNh.param("tf_map_frame",           tf_map_frame,           std::string("map"));
    privNh.param("tf_robot_frame",         tf_robot_frame,         std::string("base_footprint"));

    _tf_map_frame = tf_map_frame;
    _tf_robot_frame = tf_robot_frame;

    //init publisher
    _pub_cmd_vel = _nh.advertise<geometry_msgs::Twist>(pub_name_cmd_vel,1);
    _pubState = _nh.advertise<std_msgs::Bool>(pub_name_state,1);

    //inti subscriber
    _sub_path = _nh.subscribe(sub_name_path , 1, &PathControl::subPath_callback, this);
    //_sub_pose = _nh.subscribe(sub_name_pose , 1, &PathControl::subPose_callback, this);
    _sub_em_stop = _nh.subscribe(sub_name_em_stop, 1, &PathControl::subEmStop_callback, this);
    _sub_pause = _nh.subscribe(sub_name_pause, 1, &PathControl::subPause_callback, this);

    _pathAnalyser = new analyser::SimpleAnalyser(config_file_analyser);
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
          _tf_listnener.waitForTransform(_tf_map_frame, _tf_robot_frame, time, ros::Duration(20));
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

    //only subscriber

    while(ros::ok())
    {
       //do stuff;

       this->doPathControl();

       ros::spinOnce();
       _rate->sleep();
    }
}

void PathControl::doPathControl(void)
{
   if(_em_stop)
   {
      ROS_WARN("EMERNGENCY_STOP activated!!!! moving canceld");

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
      //_tf_listnener.waitForTransform(_tf_map_frame, _tf_robot_frame, time, ros::Duration(2));
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

   //pose.orientation.y() *= -1;
   //ROS_INFO("pos: (%f, %f, %f)", pose.position.x(),pose.position.y(),pose.position.z());
   //ROS_INFO("ori: (%f, %f, %f)", pose.orientation.x(),pose.orientation.y(),pose.orientation.z());

   //get diff scale
   analyser::diff_scale diff_scale = _pathAnalyser->analyse(pose);
   //ROS_INFO("controller _> diff_scale: (%f, %f)", diff_scale.linear, diff_scale.angular);



   //controll diffscale
   controller::velocity vel = _controller->control(diff_scale.linear, diff_scale.angular);
   //ROS_INFO("controller _> vel: (%f, %f)", vel.linear, vel.angular);
   //set twist msg
   geometry_msgs::Twist msgTwist;

   msgTwist.angular.z = vel.angular;
   msgTwist.linear.x = vel.linear;
   //publish Twist:
   analyser::info pathInfo = _pathAnalyser->getInfo();
   std_msgs::Bool reachedTarget;
   reachedTarget.data = pathInfo.reached_final_goal;

//   if(pathInfo.reached_final_goal)
//   {
//      //_enable_analyse = false;
//   }

   if(_pause)
   {
      msgTwist.angular.z = 0;
      msgTwist.linear.x = 0;
   }

   _pubState.publish(reachedTarget);
   _pub_cmd_vel.publish(msgTwist);
}

void PathControl::subPath_callback(const nav_msgs::Path& msg)
{
   std::vector<analyser::pose> path;
   ROS_INFO("GOT Path");

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
      path.push_back(tmp_pose);
   }

   //prove last path element of nan
   if(msg.poses[msg.poses.size() - 1].pose.orientation.w == std::numeric_limits<double>::quiet_NaN() ||
      msg.poses[msg.poses.size() - 1].pose.orientation.x == std::numeric_limits<double>::quiet_NaN() ||
      msg.poses[msg.poses.size() - 1].pose.orientation.y == std::numeric_limits<double>::quiet_NaN() ||
      msg.poses[msg.poses.size() - 1].pose.orientation.z == std::numeric_limits<double>::quiet_NaN()    )
   {
      //if none is nan ... than set to norotate
      ROS_INFO("Disable Rotate");
      _pathAnalyser->setDoEndRotate(false);
   }
   else
   {
      ROS_INFO("Enable Rotate");
      _pathAnalyser->setDoEndRotate(true);
   }


   _pathAnalyser->setPath(path);
   ROS_INFO("Enable analyser...");
   _enable_analyse = true;
}

void PathControl::subEmStop_callback(const std_msgs::Bool& msg)
{
   _em_stop = msg.data;
}

void PathControl::subPause_callback(const std_msgs::Bool& msg)
{
   _pause = msg.data;
}
