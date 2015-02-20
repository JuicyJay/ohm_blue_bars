
#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <limits>

#include "PathAnalyser/PathAnalyser_base.h"
#include "Controller/Controller_base.h"

using namespace Eigen;

class PathControl
{
private:    //dataelements
    unsigned int _loopRate;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    ros::Publisher _pub_cmd_vel;
    ros::Publisher _pubState;
    ros::Subscriber _sub_path;
    //ros::Subscriber _sub_pose;
    ros::Subscriber _sub_em_stop;
    ros::Subscriber _sub_pause;

    tf::TransformListener _tf_listnener;

    //path
    //std::vector<Vector3d> _path;
    //Vector3d _pose;

    analyser::PathAnalyser_base* _pathAnalyser;
    controller::Controller_base* _controller;


    bool _enable_analyse;

    bool _pause;
    bool _em_stop;

    std::string _tf_map_frame;
    std::string _tf_robot_frame;

public:
    PathControl();
    virtual ~PathControl();

    /**
     * @fn void start(const unsigned int frames = 10)
     *
     * @brief
     *
     *
     * @param[in] const unsigned int rate  ->  rate of the working loop in [1/s]
     *
     *
     * @return  void
     */
    void start(const unsigned int rate = 100);

private:    //functions

    /**
     * @fn void run()
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();

    /**
     * @brief working loop for computing twist
     *
     * @param msg -> ros msg
     */
    void doPathControl(void);

    /**
     * @brief callback for next path
     *
     * @param msg -> ros msg
     */
    void subPath_callback(const nav_msgs::Path& msg);

    /**
     * @brief if msg == true... then emergency_stop is called -> cancel moving.... need new path
     *
     * @param msg
     */
    void subEmStop_callback(const std_msgs::Bool& msg);

    /**
     * @brief if msg == true... then pause is activated.... stop moving untill msg == false
     *
     * @param msg
     */
    void subPause_callback(const std_msgs::Bool& msg);
};

#endif /* TEMPLATE_H_ */
