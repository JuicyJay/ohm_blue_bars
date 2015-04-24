#include "TargetFactory.h"

namespace {
const float POSE_POSE_DISTANCE  = 0.4f;
const float POSE_ROBOT_DISTANCE = 0.7f;
}

void TargetFactory::create(const std::vector<Wall>& walls)
{
    std::list<Target*> targets;

    for (std::vector<Wall>::const_iterator wall(walls.begin()); wall < walls.end(); ++wall)
    {
        /* Calc the number of poses for one of the both sides. */
        const unsigned int numPoses = wall->length() * 0.5f / POSE_POSE_DISTANCE;
        const Eigen::Vector3f center(wall->center().x(), wall->center().y(), 0.5f);
        const Eigen::Vector3f n(Eigen::Vector3f(wall->model().n().x(), wall->model().n().y(), 0.0f));
        const Eigen::Vector3f v(Eigen::Vector3f(wall->model().r().x(), wall->model().r().y(), 0.0f));

	Target* target = new Target(Pose(center + n * POSE_ROBOT_DISTANCE, -n));
	target->takeId();
        targets.push_back(target);

        for (unsigned int i = 1; i < numPoses; ++i)
        {
            Target* targetBack = new Target(Pose(center + v * POSE_POSE_DISTANCE * static_cast<float>(i) +
                                                 n * POSE_ROBOT_DISTANCE, -n));
            Target* targetFront = new Target(Pose(center - v * POSE_POSE_DISTANCE * static_cast<float>(i) +
                                                  n * POSE_ROBOT_DISTANCE, -n));

            targetBack->takeId();
            targetFront->takeId();
            targets.push_back(targetBack);
            targets.push_front(targetFront);
        }
    }

    _targets.resize(targets.size());
    std::copy(targets.begin(), targets.end(), _targets.begin());
}

