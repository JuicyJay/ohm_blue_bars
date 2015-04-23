#include "FoundVictimCandidate.h"
#include "../Context.h"
#include "Explore.h"
#include "Inspect.h"
#include "Drive.h"
#include "../GetTransformation.h"

#include <std_msgs/String.h>
#include <ohm_actors/SensorHeadMode.h>

#include <Eigen/Geometry>

namespace autonohm {

FoundVictimCandidate::FoundVictimCandidate(const ohm_perception::Victim& goal)
    : _nh(autonohm::Context::getInstance()->getNodeHandle()),
      _goal(goal)
{
    /* Debug publications. */
    ROS_INFO("New state is FoundVictimCandidate.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "found victim candidate";
    _state_pub.publish(msg);


    /* Change mode of the sensor head to look at point. */
    ohm_actors::SensorHeadMode mode;
    mode.request.mode = ohm_actors::SensorHeadMode::Request::LOOK_AT_POINT;
    _srvSensorHeadMode = _nh->serviceClient<ohm_actors::SensorHeadMode>("/georg/sensor_head/mode");
    _pubGoal = _nh->advertise<geometry_msgs::Point>("/georg/goal/sensor_head", 2);

    if (!_srvSensorHeadMode.call(mode))
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the service sensor head mode.");


    /* Service for to can call the victim stack node. */
    _srvVictimStack = _nh->serviceClient<ohm_perception::GetVictim>("/victim/get_victim");

    /* And create a subcriber for roboter movements. */
    _subMoveRobot = _nh->subscribe("/georg/move_direction", 2, &FoundVictimCandidate::callbackMoveRobot, this);

    /* Take a time stamp. */
    _stamp = ros::Time::now();
}

FoundVictimCandidate::~FoundVictimCandidate(void)
{

}

void FoundVictimCandidate::process(void)
{
//    if ((ros::Time::now() - _stamp).toSec() > 20.0f)
//    {
//        ROS_INFO("Found Victim Candidate state already lives 20 seconds. Now its time to kill it.");
//
//        /* Set the state after and kill myself.*/
//        Context::getInstance()->setState(new Inspect);
//        delete this;
//        return;
//    }

    _pubGoal.publish(_goal.pose.position);


    /* Check if the victim was classified. */
    ohm_perception::GetVictim stack;
    stack.request.id = _goal.id;

    if (!_srvVictimStack.call(stack))
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the service GetVictim of the victim stack.");
        return;
    }
    if (stack.response.victim.checked)
    {
        /* Set the state after and kill myself.*/
      //        Context::getInstance()->setState(new Inspect);
      Context::getInstance()->setState(new Explore);
      
        delete this;
        return;
    }
}

void FoundVictimCandidate::callbackMoveRobot(const ohm_autonomy::MoveRobot& msg)
{
    const Eigen::Quaternionf orientation(_goal.pose.orientation.w,
                                   _goal.pose.orientation.x,
                                   _goal.pose.orientation.y,
                                   _goal.pose.orientation.z);
    const Eigen::Vector3f n(orientation * Eigen::Vector3f::UnitX());
    GetTransformation* tf = GetTransformation::instance();
    tf->lookUpTransform("map", "georg/base");

    switch (msg.direction)
    {
    case ohm_autonomy::MoveRobot::LEFT:
        {
            const Eigen::Vector3f v(orientation * Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ()) *
                                    Eigen::Vector3f::UnitX());
            const Eigen::Vector3f p(tf->position() + v * 0.3f);
            geometry_msgs::Point position;

            position.x = p.x();
            position.y = p.y();
            position.z = p.z();
            Context::getInstance()->setState(new Drive(position, _goal.pose.orientation, this));
        }
        break;

    case ohm_autonomy::MoveRobot::RIGHT:
        {
            const Eigen::Vector3f v(orientation * Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ()) *
                                    Eigen::Vector3f::UnitX());
            const Eigen::Vector3f p(tf->position() + v * 0.3f);
            geometry_msgs::Point position;

            position.x = p.x();
            position.y = p.y();
            position.z = p.z();
            Context::getInstance()->setState(new Drive(position, _goal.pose.orientation, this));
        }
        break;

    case ohm_autonomy::MoveRobot::CLOSER:
        break;

    case ohm_autonomy::MoveRobot::FARTHER:
        break;

    default:
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": direction not implemented.");
        break;
    }
}

} // end namespace autonohm

