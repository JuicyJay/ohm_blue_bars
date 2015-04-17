/************************************************************************************************************
 * Struct Pose: At the moment only for copying data to the ROS framework.
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___POSE_H___
#define ___POSE_H___

#include <geometry_msgs/Pose.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose(void) { }

    Pose(const Eigen::Vector3f& pos, const Eigen::Vector3f& v)
        : position(pos)
    {
        orientation.setFromTwoVectors(Eigen::Vector3f(1.0f, 0.0f, 0.0f), v);
    }

    Pose(const geometry_msgs::Pose& pose)
        : position(pose.position.x, pose.position.y, pose.position.z),
          orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
    {

    }

    geometry_msgs::Pose toRos(void) const
    {
        geometry_msgs::Pose pose;

        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();

        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        pose.orientation.w = orientation.w();

        return pose;
    }

    Eigen::Vector3f    position;
    Eigen::Quaternionf orientation;
};

#endif
