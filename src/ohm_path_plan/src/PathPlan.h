
#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#include <boost/timer.hpp>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ohm_path_plan/MoveToAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>

#include <obvision/planning/AStar.h>
#include <obcore/base/tools.h>


class PathPlan
{
private:    //dataelements
    unsigned int _loopRate;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    ros::Publisher _pubPath;
    ros::Subscriber _subMap;
    ros::Subscriber _subState;

    //action stuff
    actionlib::SimpleActionServer<ohm_path_plan::MoveToAction>* _actionMoveTo;
    ohm_path_plan::MoveToFeedback _actionMoveTo_feedback;
    ohm_path_plan::MoveToResult _actionMoveTo_result;

    geometry_msgs::PoseStamped _pathStart;
    geometry_msgs::PoseStamped _pathEnd;

    nav_msgs::OccupancyGrid _map;


    double _robotRadius;
    std::string _frame_id;

public:
    PathPlan();
    virtual ~PathPlan();

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
    void start(const unsigned int rate = 10);

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

    void goalCallback();

    void subReached_callback(const std_msgs::Bool& msg);

    void subCallback_map(const nav_msgs::OccupancyGrid& msg);
};

#endif /* TEMPLATE_H_ */
