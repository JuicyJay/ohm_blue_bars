
#include "OhmCat.h"

namespace{
const double HANKY_SIZE = 0.40;
}

OhmCat::OhmCat() : _rate(0)
{
    _loopRate = 0;


    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_vel;
    std::string pub_event;
    std::string sub_vel;
    std::string srv_slam_ctrl;
    std::string robot_foot_print;
    std::string robot_link;
    double max_pitch;  //max degree of yaw
    double max_roll; //max degree of roll
    double time_to_stop;
    double time_to_move;
    double time_to_ok;
    double time_move;
    double time_next_move;
    double fix_move_vel;
    //int int_val;
    
    privNh.param("pub_vel",                 pub_vel,         std::string("vel/teleop"));
    privNh.param("pub_event",               pub_event,       std::string("/georg/event"));
    privNh.param("sub_vel",                 sub_vel,         std::string("path_control/vel/teleop"));
    privNh.param("srv_slam_ctrl",           srv_slam_ctrl,   std::string("/georg/node_control"));
    privNh.param("robot_foot_print",        robot_foot_print,    std::string("georg/center"));
    privNh.param("robot_link",              robot_link,      std::string("georg/base"));
    privNh.param<double>("max_pitch",       max_pitch,       40.0);
    privNh.param<double>("max_roll",        max_roll,        40.0);
    privNh.param<double>("time_to_stop",    time_to_stop,    0.2);
    privNh.param<double>("time_to_move",    time_to_move,    0.2);
    privNh.param<double>("time_to_ok",      time_to_ok,      0.5);
    privNh.param<double>("time_move",       time_move,       1.0);
    privNh.param<double>("time_next_move",  time_next_move,  5.0);
    privNh.param<double>("fix_move_vel",    fix_move_vel,    0.2);

    _robot_foot_print = robot_foot_print;
    _robot_link = robot_link;
    _max_pitch = (max_pitch / 180) * M_PI;
    _max_roll = (max_roll / 180) * M_PI;
    _duration_ok   = ros::Duration(time_to_ok);
    _duration_stop = ros::Duration(time_to_stop);
    _duration_to_move = ros::Duration(time_to_move);
    _duration_move = ros::Duration(time_move);
    _duration_next_move = ros::Duration(time_next_move);
    _fix_move_vel = fix_move_vel;

    //init publisher
    _pubVel   = _nh.advertise<geometry_msgs::Twist>(pub_vel,1);
    _pubEvent = _nh.advertise<ohm_common::RobotEvent>(pub_event, 20);
    _pubHanky = _nh.advertise<ohm_common::Obstacle>("/georg/path_plan/add_obstacle",20);

    //inti subscriber
    _subVel = _nh.subscribe(sub_vel, 1, &OhmCat::subVel_callback, this);

    //init service
    _srv_slam = _nh.serviceClient<ohm_srvs::NodeControl>(srv_slam_ctrl);
    _srv_slam.waitForExistence();

    _state = ohm_cat::OK;
    _state_old = ohm_cat::OK;

    _direction = -1;

    _cnt = 0;

    //init times:
    _time_last_ok = ros::Time::now();
    _time_last_notOk = ros::Time::now();
    _time_last_move = ros::Time::now();

    //ROS_INFO_STREAM("Time_last_ok: " << _time_last_ok);
    //ROS_INFO_STREAM("Time_last_notOk: " << _time_last_notOk);
}

OhmCat::~OhmCat()
{
    delete _rate;
}

void OhmCat::start(const unsigned int rate)
{
    delete _rate;
    _loopRate = rate;
    _rate = new ros::Rate(_loopRate);

    //wait for first transform
    bool rdy = false;
    do{
       try {
          ros::Time time = ros::Time::now();
          _tf_listnener.waitForTransform(_robot_foot_print, _robot_link, time, ros::Duration(1));
          rdy = true;

       } catch (tf::TransformException& e)
       {
          ROS_ERROR("ohm_path_control -> Exeption at tf: %s", e.what());
          continue;
       }
    }while(!rdy && ros::ok());

    ROS_INFO("ohm_cat -> Got first Transform... now rdy");

    this->run();
}

void OhmCat::run()
{
    unsigned int cnt = 0;

    while(ros::ok())
    {
        //do stuff;

        //publish data;
        //_pub.publish(msg);

       //get tf
       tf::StampedTransform tf;
       try {
          ros::Time time = ros::Time(0);
          _tf_listnener.lookupTransform(_robot_foot_print, _robot_link, time, tf);

       } catch (tf::TransformException& e)
       {
          ROS_ERROR("ohm_cat -> Exeption at tf: %s", e.what());
          _state = ohm_cat::STOP;
          _time_last_notOk = ros::Time::now();

          _rate->sleep();
          ros::spinOnce();
          continue;
       }

       Eigen::Quaternion<double> tmp_q(tf.getRotation().w(),
                                       tf.getRotation().x(),
                                       tf.getRotation().y(),
                                       tf.getRotation().z() );

       //get Pitch rot (rot y-asxis)
       Eigen::Vector3d ez(0,0,1);
       Eigen::Vector3d v = tmp_q * ez;
       v.y() = 0;
       double pitch = ::acos((v.dot(ez))/v.norm());
       if(v.x() < 0)
          pitch *= -1;

       //get Roll rot (rot x-axis)
       Eigen::Vector3d vr = tmp_q * ez;
       vr.x() = 0;
       double roll = ::acos((vr.dot(ez))/vr.norm());
       if(vr.y() < 0)
          roll *= -1;

       //ROS_INFO("Pitch:  %f     ,   Roll:  %f", pitch / M_PI * 180, roll / M_PI * 180);

       //todo prove if move mode check is needed
       if(_state == ohm_cat::FIX_MOVE)
       {
          //prove of exit fix mode.. after time
          if(ros::Time::now() - _time_last_notOk > _duration_move)
          {
             _time_last_move = ros::Time::now();
             _state = ohm_cat::STOP;
             //if(_state_old == ohm_cat::FIX_MOVE && _state == ohm_cat::STOP)
                //this->setLaserinterupt(false);
             _state_old = ohm_cat::STOP;
             ROS_INFO("ohm_cat -> now Exit Fix Mode");
          }
       }
       else
       {

          //detect flip
          if( (std::abs(pitch) < _max_pitch && std::abs(roll) < _max_roll) &&
              ((ros::Time::now() - _time_last_notOk) > _duration_ok)          )
          {//okay
             //ROS_INFO_STREAM("ohm_cat -> OK" << "time: " <<   (ros::Time::now() - _time_last_notOk));
             _state = ohm_cat::OK;
             //event
             if(_state_old != ohm_cat::OK && _state == ohm_cat::OK)
             {
                this->event_OK();
             }
             _state_old = ohm_cat::OK;

             _time_last_ok = ros::Time::now();
          }
          else if( (std::abs(pitch) >= _max_pitch || std::abs(roll) >= _max_roll) &&
                   ((ros::Time::now() - _time_last_ok) > _duration_stop)           )
          {//stop detected after time
             _state = ohm_cat::STOP;
             //event
             if(_state_old != ohm_cat::STOP && _state == ohm_cat::STOP)
             {
                this->event_Stop();
             }
             _state_old = ohm_cat::STOP;

             _time_last_notOk = ros::Time::now();
             //ROS_INFO_STREAM("ohm_cat -> STOP" << "time: " <<  (ros::Time::now() - _time_last_ok));
             //prove if a fix move is availible
             if( (pitch < 0) && //&& std::abs(roll) < _max_roll) &&
                 (ros::Time::now() - _time_last_ok > _duration_to_move) &&
                 (ros::Time::now() - _time_last_move > _duration_next_move)   )
             {
                _direction = -1;
                _state = ohm_cat::FIX_MOVE;

                //event
                if(_state_old != ohm_cat::FIX_MOVE && _state == ohm_cat::FIX_MOVE)
                {
                   this->event_Fix();
                }
                _state_old = ohm_cat::FIX_MOVE;

                ROS_INFO("ohm_cat -> now Fix mode: move back");
             }
             else if( (pitch > 0 ) && //&& std::abs(roll) < _max_roll) &&
                   (ros::Time::now() - _time_last_ok > _duration_to_move) &&
                   (ros::Time::now() - _time_last_move > _duration_next_move)   )
             {
                _direction = 1;
                _state = ohm_cat::FIX_MOVE;
                //event
                if(_state_old != ohm_cat::FIX_MOVE && _state == ohm_cat::FIX_MOVE)
                {
                   this->event_Fix();
                }
                _state_old = ohm_cat::FIX_MOVE;
                ROS_INFO("ohm_cat -> now Fix mode: move forward");
             }
             else
             {//stay STOP noting to do
                _state = ohm_cat::STOP;
             }
          }

       }
        ros::spinOnce();
        _rate->sleep();
    }
}

bool OhmCat::setLaserinterupt(bool value)
{
   ohm_srvs::NodeControl srv;
   srv.request.action = value ? (int8_t)srv.request.PAUSE : (int8_t)srv.request.START;

   if(_srv_slam.call(srv))
   {
      if(srv.response.accepted)
      {
         return true;
      }
   }
   else
   {
      ROS_WARN("ohm_cat -> unable to set NodeControl to slam");
   }
   ROS_WARN("ohm_cat -> unable to set NodeControl to slam: no accept");

   return false;
}

void OhmCat::subVel_callback(const geometry_msgs::Twist& msg)
{
   geometry_msgs::Twist vel = msg;

   switch (_state) {
      case ohm_cat::OK:
         //nop
         break;
      case ohm_cat::STOP:
         //set to 0
         vel.angular.x = 0;
         vel.angular.y = 0;
         vel.angular.z = 0;
         vel.linear.x = 0;
         vel.linear.y = 0;
         vel.linear.z = 0;
         break;
      case ohm_cat::FIX_MOVE:
         vel.angular.x = 0;
         vel.angular.y = 0;
         vel.angular.z = 0;
         vel.linear.x = _direction * _fix_move_vel;
         vel.linear.y = 0;
         vel.linear.z = 0;

         break;
      default:
         break;
   }
   _pubVel.publish(vel);
}

void OhmCat::event_Stop()
{
   ROS_INFO("ohm_cat -> event: Stop");

   //pub hanky
   //get posisition
   tf::StampedTransform tf;
   while(true)
   {
      try {
//         ros::Time time = NULL;
//         _tf_listnener.waitForTransform("map", "georg/center", time, ros::Duration(1));
         _tf_listnener.lookupTransform("map", "georg/center", ros::Time(0), tf);
         break;

      } catch (tf::TransformException& e)
      {
         usleep(10000);
         ROS_ERROR("ohm_cortex: FrontierExploration -> Exeption at tf: %s", e.what());
      }
   }

   std::stringstream ss;
   ss << "cat_" <<  _cnt++;

   //create obstacle
   ohm_common::Obstacle msg_hanky;
   msg_hanky.name.data = ss.str();
   msg_hanky.rect.x       = tf.getOrigin().x() - HANKY_SIZE * 0.5;
   msg_hanky.rect.y       = tf.getOrigin().y() - HANKY_SIZE * 0.5;
   msg_hanky.rect.width   = HANKY_SIZE;
   msg_hanky.rect.height  = HANKY_SIZE;

   _pubHanky.publish(msg_hanky);

   ohm_common::RobotEvent msg;
   msg.event = msg.FLIP_OVER;
   _pubEvent.publish(msg);

   this->setLaserinterupt(true);
}

void OhmCat::event_Fix()
{
   ROS_INFO("ohm_cat -> event: Fix");

}

void OhmCat::event_OK()
{
   ROS_INFO("ohm_cat -> event: OK");

   this->setLaserinterupt(false);
}

