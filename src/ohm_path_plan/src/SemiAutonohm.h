
#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <ohm_path_plan/MoveToAction.h>

#include <string>

class SemiAutonohm
{
private:    //dataelements
    unsigned int _loopRate;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    //ros::Publisher _pub;
    ros::Subscriber _subGoal;
    ros::Subscriber _subPose;

    actionlib::SimpleActionClient<ohm_path_plan::MoveToAction>* _ac;

    geometry_msgs::PoseStamped _currentPose;

public:
    SemiAutonohm();
    virtual ~SemiAutonohm();

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

    void subGoal_callback(const geometry_msgs::PoseStamped& msg);

    void subPose_callback(const geometry_msgs::PoseStamped& msg);
};

#endif /* TEMPLATE_H_ */
