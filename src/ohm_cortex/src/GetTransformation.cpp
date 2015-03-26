#include "GetTransformation.h"

#include <ros/ros.h>

GetTransformation* GetTransformation::_instance = 0;

GetTransformation::GetTransformation(void)
{

}

GetTransformation* GetTransformation::instance(void)
{
    if (_instance)
        return _instance;

    _instance = new GetTransformation;
    return _instance;
}

bool GetTransformation::lookUpTransform(const std::string& from, const std::string& to)
{
    tf::StampedTransform transform;

    try
    {
        _listener.lookupTransform(from, to, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("GetTransformation: %s", ex.what());
        return false;
    }

    _position.x() = transform.getOrigin().x();
    _position.y() = transform.getOrigin().y();
    _position.z() = transform.getOrigin().z();

    _orientation.w() = transform.getRotation().w();
    _orientation.x() = transform.getRotation().x();
    _orientation.y() = transform.getRotation().y();
    _orientation.z() = transform.getRotation().z();

    return true;
}

bool GetTransformation::waitAndLookUpTransform(const std::string& from, const std::string& to, const float sec)
{
    tf::StampedTransform transform;

    try
    {
        const ros::Time stamp(ros::Time::now());
        _listener.waitForTransform(from, to, stamp, ros::Duration(sec));
        _listener.lookupTransform(from, to, stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("GetTransformation: %s", ex.what());
        return false;
    }

    _position.x() = transform.getOrigin().x();
    _position.y() = transform.getOrigin().y();
    _position.z() = transform.getOrigin().z();

    _orientation.w() = transform.getRotation().w();
    _orientation.x() = transform.getRotation().x();
    _orientation.y() = transform.getRotation().y();
    _orientation.z() = transform.getRotation().z();

    return true;
}

