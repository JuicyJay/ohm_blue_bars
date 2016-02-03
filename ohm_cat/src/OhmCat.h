
#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#include <string>
#include <sstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <ohm_common/RobotEvent.h>
#include <ohm_common/Obstacle.h>

#include <ohm_srvs/NodeControl.h>

#include <Eigen/Dense>


using namespace Eigen;


namespace ohm_cat{

enum enumState{
   OK = 0,
   STOP,
   FIX_MOVE
};

}

class OhmCat
{
private:    //dataelements
    unsigned int _loopRate;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    ros::Publisher _pubVel;
    ros::Publisher _pubEvent;
    ros::Publisher _pubHanky;
    ros::Subscriber _subVel;

    ros::ServiceClient _srv_slam;

    tf::TransformListener _tf_listnener;

    ros::Time _time_last_ok;
    ros::Time _time_last_notOk;
    ros::Time _time_last_move;
    ros::Duration _duration_ok;
    ros::Duration _duration_to_move;
    ros::Duration _duration_stop;
    ros::Duration _duration_move;
    ros::Duration _duration_next_move;

    std::string _robot_foot_print; ///none roated robot center frame
    std::string _robot_link;   ///rotated robot frame(with imu)

    ohm_cat::enumState _state;
    ohm_cat::enumState _state_old;

    double _max_roll;
    double _max_pitch;

    double _fix_move_vel;

    int _direction;

    int _cnt;

public:
    OhmCat();
    virtual ~OhmCat();

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


    bool setLaserinterupt(bool value);


    //void subCallback(const ROS_PACK::MESSAGE& msg);
    void subVel_callback(const geometry_msgs::Twist& msg);

    void event_Stop();
    void event_Fix();
    void event_OK();


};

#endif /* TEMPLATE_H_ */
