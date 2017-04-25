
#ifndef FRONTIER_BASIC_H_
#define FRONTIER_BASIC_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseArray.h>

#include <ohm_apps_msgs/NodeControl.h>
#include <rona_msgs/State.h>


class Frontier_basic
{
private:    //dataelements
    ros::NodeHandle _nh;

    ros::Publisher _pubTarget;

    ros::Subscriber _subFrontiers;
    ros::Subscriber _subMoveState;

    ros::ServiceClient _srv_ctrl_frontier;

    ros::Timer _loopTimer;
public:
    Frontier_basic();
    virtual ~Frontier_basic();

    /**
     *
     * @brief
     *
     * @return  void
     */
    void start(double duration = 0.01);

private:    //functions

    /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();


    void loop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);


    void subFrontier_callback(const geometry_msgs::PoseArray& msg);
    void subState_callback(const rona_msgs::State& msg);
};

#endif /* FRONTIER_BASIC_H_ */
