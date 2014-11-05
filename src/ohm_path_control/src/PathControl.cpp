
#include "PathControl.h"
#include "PathAnalyser/SimpleAnalyser.h"
#include "Controller/ParabolaTransfere.h"

PathControl::PathControl() : _rate(0)
{
    _loopRate = 0;

    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_name_cmd_vel;
    std::string sub_name_path;
    std::string sub_name_pose;
    std::string config_file_controller;
    std::string config_file_analyser;
    
    privNh.param("pub_name_cmd_vel",       pub_name_cmd_vel,       std::string("robot0/cmd_vel"));
    privNh.param("sub_name_path",          sub_name_path,          std::string("path"));
    privNh.param("sub_name_pose",          sub_name_pose,          std::string("robot0/pose"));
    privNh.param("config_file_controller", config_file_controller, std::string("/home/m1ch1/workspace/ros/ohm_autonomy/src/ohm_path_control/config/controller.xml"));
    privNh.param("config_file_analyser",   config_file_analyser,   std::string("/home/m1ch1/workspace/ros/ohm_autonomy/src/ohm_path_control/config/analyser.xml"));

    //init publisher
    _pub_cmd_vel = _nh.advertise<geometry_msgs::Twist>(pub_name_cmd_vel,1);

    //inti subscriber
    _sub_path = _nh.subscribe(sub_name_path , 1, &PathControl::subPath_callback, this);
    _sub_pose = _nh.subscribe(sub_name_pose , 1, &PathControl::subPose_callback, this);

    _pathAnalyser = new analyser::SimpleAnalyser(config_file_analyser);
    _controller = new controller::ParabolaTransfere(config_file_controller);

    _enable_analyse = false;
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
    this->run();
}

void PathControl::run()
{
    unsigned int cnt = 0;

    //only subscriber
    ros::spin();

//    while(ros::ok())
//    {
//        //do stuff;
//
//        //publish data;
//        //_pub.publish(msg);
//
//        ros::spinOnce();
//        _rate->sleep();
//    }
}

void PathControl::subPose_callback(const geometry_msgs::PoseStamped& msg)
{
   if(!_enable_analyse)
   {
      return;
   }

   analyser::pose pose;
   pose.position = Vector3d(msg.pose.position.x, msg.pose.position.y, 0);
   Quaternion<double> tmp_q(msg.pose.orientation.w,
                            msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z);
   pose.orientation = analyser::PathAnalyser_base::quaternion_to_orientationVec(tmp_q);

   //get diff scale
   analyser::diff_scale diff_scale = _pathAnalyser->analyse(pose);

   //controll diffscale
   controller::velocity vel = _controller->control(diff_scale.linear, diff_scale.angular);

   //set twist msg
   geometry_msgs::Twist msgTwist;

   msgTwist.angular.z = vel.angular;
   msgTwist.linear.x = vel.linear;
   //publish Twist:
   _pub_cmd_vel.publish(msgTwist);
}

void PathControl::subPath_callback(const nav_msgs::Path& msg)
{
   std::vector<analyser::pose> path;

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
   _pathAnalyser->setPath(path);
   _enable_analyse = true;
}
