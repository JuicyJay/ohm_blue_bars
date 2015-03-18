#include "Target.h"

namespace {
const float POSE_POSE_DISTANCE  = 0.4f;
const float POSE_ROBOT_DISTANCE = 0.6f;
}

#include <iostream>

Target::Target(const Wall& wall)
    : _id(wall.id()),
      _inspected(false)
{
    /* Calc the number of poses for one of the both sides. */
    const unsigned int numPoses = wall.length() * 0.5f / POSE_POSE_DISTANCE;
//    std::cout << "length = " << wall.length() << std::endl;
//    std::cout << "poses = " << poses << std::endl;
    const Eigen::Vector3f center(wall.center().x(), wall.center().y(), 0.5f);
    const Eigen::Vector3f n(Eigen::Vector3f(wall.model().n().x(), wall.model().n().y(), 0.0f));
    const Eigen::Vector3f v(Eigen::Vector3f(wall.model().r().x(), wall.model().r().y(), 0.0f));

    std::list<Pose> poses;

    poses.push_back(Pose(center + n * POSE_ROBOT_DISTANCE, -n));

    for (unsigned int i = 1; i < numPoses; ++i)
    {
        poses.push_back (Pose(center + v * POSE_POSE_DISTANCE * static_cast<float>(i) +
			      n * POSE_ROBOT_DISTANCE, -n));
        poses.push_front(Pose(center - v * POSE_POSE_DISTANCE * static_cast<float>(i) +
			      n * POSE_ROBOT_DISTANCE, -n));
    }

    _poses.resize(poses.size());
    std::copy(poses.begin(), poses.end(), _poses.begin());
}
