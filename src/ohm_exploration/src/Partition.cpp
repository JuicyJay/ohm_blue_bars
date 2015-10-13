#include "Partition.h"
#include "Target.h"

#include <cstdlib>
#include <limits>

#include <ohm_path_plan/PlanPaths.h>

int Partition::s_id = 0;
float Partition::s_maxDistanceFromOrigin = 20.0f;
float Partition::s_minDistanceFromOrigin = 4.5f;

Partition::Partition(void)
    : _initialized(false),
      _id(-1),
      _xMin(0.0f),
      _yMin(0.0f),
      _xMax(0.0f),
      _yMax(0.0f),
      _center(0.0f, 0.0f)
{

}

Partition::Partition(const float xMin, const float yMin, const float size)
    : _initialized(true),
      _id(++s_id),
      _xMin(xMin),
      _yMin(yMin),
      _xMax(xMin + size),
      _yMax(yMin + size),
      _center((_xMax - _xMin) * 0.5f, (_yMax - _yMin) * 0.5f)
{

}

Partition::Partition(const Eigen::Vector2f& center, const float size)
    : _initialized(true),
      _id(++s_id),
      _xMin(center.x() - size * 0.5f),
      _yMin(center.y() - size * 0.5f),
      _xMax(center.x() + size * 0.5f),
      _yMax(center.y() + size * 0.5f),
      _center(center)
{

}

bool Partition::insert(Target* target)
{
    if (target->pose().position.x() >= _xMin && target->pose().position.x() <= _xMax &&
        target->pose().position.y() >= _yMin && target->pose().position.y() <= _yMax)
    {
        _targets.push_back(target);
        return true;
    }

    return false;
}

bool Partition::insert(std::vector<Target*>& targets)
{
    bool ok = true;

    for(std::vector<Target*>::iterator target(targets.begin()); target < targets.end(); ++target)
        ok = ok & this->insert(*target);

    return ok;
}

Target* Partition::target(void)
{
    int min = -1;

    for (unsigned int i = 0; i < _targets.size(); ++i)
    {
      ROS_INFO("distance from orign %f", _targets[i]->distanceFromOrigin());
      if (!_targets[i]->inspected() && _targets[i]->distanceFromOrigin() < s_maxDistanceFromOrigin
	  && _targets[i]->distanceFromOrigin() > s_minDistanceFromOrigin)
        {
            if (min < 0) min = i;
            if (_targets[i]->distance() < _targets[min]->distance()) min = i;
        }
    }

    if (min < 0) return 0;

    return _targets[min];
}

int Partition::numUninspectedTargets(void) const
{
    int num = 0;

    for (std::vector<Target*>::const_iterator target(_targets.begin()); target < _targets.end(); ++target)
        if (!(**target).inspected())
            ++num;

    return num;
}

int Partition::numValidTargets(void) const
{
    int num = 0;

    for (std::vector<Target*>::const_iterator target(_targets.begin()); target < _targets.end(); ++target)
    {
        if (!(**target).inspected() && (**target).distanceFromOrigin() < s_maxDistanceFromOrigin
	    && (**target).distanceFromOrigin() > s_minDistanceFromOrigin)
      {
	++num;
      }
    }

    return num;
}

void Partition::estimateDistances(ros::ServiceClient& client, const Pose& robot)
{
    ohm_path_plan::PlanPaths paths;
    paths.request.origin = robot.toRos();

    for (std::vector<Target*>::const_iterator target(_targets.begin()); target < _targets.end(); ++target)
        paths.request.targets.push_back((**target).pose().toRos());


    if (!client.call(paths))
    {
        ROS_ERROR("Cannot call the path plan node.");
        return;
    }
    if (paths.response.lengths.size() != _targets.size())
    {
        ROS_ERROR("Not enough distances received from the path plan node.");
        return;
    }


    std::vector<double>::const_iterator distance(paths.response.lengths.begin());
    for (std::vector<Target*>::iterator target(_targets.begin()); target < _targets.end(); ++target, ++distance)
        (**target).setDistance(*distance < 0 ? std::numeric_limits<float>::max() : *distance);
}

void Partition::printAllTargets(void) const
{
  ROS_INFO_STREAM(__PRETTY_FUNCTION__);

    for (std::vector<Target*>::const_iterator target(_targets.begin()); target < _targets.end(); ++target)
        ROS_INFO_STREAM(**target);
}

visualization_msgs::Marker Partition::getMarkerMsg(void) const
{
    visualization_msgs::Marker msg;

    msg.header.frame_id = "map";
    msg.header.stamp    = ros::Time();
    msg.ns              = "";
    msg.id              = _id;
    msg.type            = visualization_msgs::Marker::LINE_LIST;
    msg.action          = visualization_msgs::Marker::ADD;

    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;

    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    msg.scale.x = 0.05;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.color.a = 1.0;
//    msg.color.r = static_cast<float>(::rand()) / static_cast<float>(std::numeric_limits<int>::max());
//    msg.color.g = static_cast<float>(::rand()) / static_cast<float>(std::numeric_limits<int>::max());
//    msg.color.b = static_cast<float>(::rand()) / static_cast<float>(std::numeric_limits<int>::max());
    msg.color.r = 1.0f;
    msg.color.g = 0.0f;
    msg.color.b = 0.0f;

    /* Set the borderline. */
    msg.points.resize(8);

    msg.points[0].x = _xMin;
    msg.points[0].y = _yMin;
    msg.points[0].z = 0.0;
    msg.points[1].x = _xMax;
    msg.points[1].y = _yMin;
    msg.points[1].z = 0.0;

    msg.points[2].x = _xMax;
    msg.points[2].y = _yMin;
    msg.points[2].z = 0.0;
    msg.points[3].x = _xMax;
    msg.points[3].y = _yMax;
    msg.points[3].z = 0.0;

    msg.points[4].x = _xMax;
    msg.points[4].y = _yMax;
    msg.points[4].z = 0.0;
    msg.points[5].x = _xMin;
    msg.points[5].y = _yMax;
    msg.points[5].z = 0.0;

    msg.points[6].x = _xMin;
    msg.points[6].y = _yMax;
    msg.points[6].z = 0.0;
    msg.points[7].x = _xMin;
    msg.points[7].y = _yMin;
    msg.points[7].z = 0.0;

    return msg;
}
