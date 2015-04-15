/************************************************************************************************************
 * Class Partition:
 *
 *  Created on: 13.04.2015
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___PARTITION_H___
#define ___PARTITION_H___

#include <vector>

#include <Eigen/Core>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include "Pose.h"

class Target;

class Partition
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Partition(void);
    Partition(const float xMin, const float yMin, const float size);
    Partition(const Eigen::Vector2f& center, const float size);

    bool insert(Target* target);
    bool insert(std::vector<Target*>& targets);
    Target* target(void);
    int numUninspectedTargets(void) const;
    int numValidTargets(void) const;
    const Eigen::Vector2f& center(void) const { return _center; }

    void estimateDistances(ros::ServiceClient& client, const Pose& robot);

    visualization_msgs::Marker getMarkerMsg(void) const;

private:
    bool _initialized;
    int _id;
    float _xMin;
    float _yMin;
    float _xMax;
    float _yMax;
    Eigen::Vector2f _center;

    std::vector<Target*> _targets;

    static int s_id;
    static float s_maxDistanceFromOrigin;
};

#endif
