#ifndef ___POSE_H___
#define ___POSE_H___

#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose(const Eigen::Vector3f& pos, const Eigen::Vector3f& orient) : position(pos), orientation(orient) { }

    geometry_msgs::Pose toRos(void) const
    {
        geometry_msgs::Pose pose;

        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();

        Eigen::Quaternionf rotation;

        rotation.setFromTwoVectors(Eigen::Vector3f(1.0f, 0.0f, 0.0f), orientation);
        pose.orientation.x = rotation.x();
        pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z();
        pose.orientation.w = rotation.w();

        return pose;
    }

    Eigen::Vector3f position;
    Eigen::Vector3f orientation;
};

#endif
