/***********************************************************************************************************
 * mission-planner-node
 *
 *  Created on: 29.03.2015
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ***********************************************************************************************************/
#include <ros/ros.h>
#include <ohm_common/Rect.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>

ros::Publisher _pubRoi;
tf::TransformListener* _listener = 0;
std::string _tfRobot;

bool callbackNextSection(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    tf::StampedTransform T;

    try
    {
        ros::Time stamp(ros::Time::now());
        _listener.waitForTransform("map", _tfRobot, stamp, ros::Duration(1.0));
        _listener.lookupTransform ("map", _tfRobot, stamp, T);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return false;
    }


    /* Calculate the next roi the robot should explore. */


    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mission_planner");
    ros::NodeHandle para("~");
    ros::NodeHandle nh;
    _listener = new tf::TransformListener;

    std::string value;
    para.param<std::string>("service_next_section", value, "mission_planner/trigger/next_section");
    ros::ServiceServer srvNextSection(nh.advertiseService(value, callbackNextSection));
    para.param<std::string>("tf_robot", value, "georg/base");

    _pubRoi = nh.advertise<ohm_common::Rect>("exploration/set_roi", 2);

    ros::spin();

    delete _listener;
}
