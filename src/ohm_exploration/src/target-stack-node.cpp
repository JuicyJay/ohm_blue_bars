#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#include <list>

#include "Wall.h"
#include "Target.h"
#include "ohm_exploration/WallArray.h"
#include "ohm_exploration/MarkTarget.h"
#include "ohm_exploration/GetTarget.h"
#include "ohm_path_plan/PlanPaths.h"

std::list<Wall> _haveToInspect;
std::vector<Target> _targets;
ros::ServiceClient _srvPlanPaths;
tf::TransformListener* _listener = 0;
std::string _tfSource, _tfTarget;

void callbackWalls(const ohm_exploration::WallArray& walls)
{
    /* Use the center of the walls to create targets and get the corresponding lenghts. */
    ohm_path_plan::PlanPaths paths;
    std::vector<Wall> receivedWalls;

    for (std::vector<ohm_exploration::Wall>::const_iterator wall(walls.walls.begin());
         wall < walls.walls.end();
         ++wall)
    {
        const Eigen::Vector3f center(wall->center.x, wall->center.y, 0.0f);
        const Eigen::Vector3f n(wall->n.x, wall->n.y, 0.0f);
        const Pose pose(center + n * 0.5f, -n);

        paths.request.targets.push_back(pose.toRos());
        receivedWalls.push_back(Wall(*wall));
    }


    /* Get the current robot pose. */
    tf::StampedTransform transform;

    try
    {
        _listener->lookupTransform(_tfSource, _tfTarget, ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ROS_INFO("Will use coordinate (0, 0, 0).");
    }

    paths.request.origin.position.x = transform.getOrigin().x();
    paths.request.origin.position.y = transform.getOrigin().y();
    paths.request.origin.position.z = transform.getOrigin().z();

    paths.request.origin.orientation.w = transform.getRotation().w();
    paths.request.origin.orientation.x = transform.getRotation().x();
    paths.request.origin.orientation.y = transform.getRotation().y();
    paths.request.origin.orientation.z = transform.getRotation().z();


    /* Ask the navigation node for the distances of each path. */
    if (_srvPlanPaths.call(paths))
    {
        for (unsigned int i = 0; i < paths.response.lengths.size(); ++i)
	{
            receivedWalls[i].setDistance(paths.response.lengths[i] < 0.0 ? std::numeric_limits<float>::max() :  paths.response.lengths[i]);
	    ROS_INFO("distance to wall %u is %f.", i, paths.response.lengths[i]);
	}

        std::sort(receivedWalls.begin(), receivedWalls.end());

	for (unsigned int i = 0; i < receivedWalls.size(); ++i)
	  ROS_INFO("distance to wall %u is %f.", i, receivedWalls[i].distance());
    }
    else
    {
        ROS_ERROR("Cannot call the navigation node.");
        ROS_INFO("Will use the unsorted targets.");
    }


    /* Put all targets to the have to inspect list */
    _haveToInspect.insert(_haveToInspect.end(), receivedWalls.begin(), receivedWalls.end());

//    for (std::vector<ohm_exploration::Wall>::const_iterator wall(walls.walls.begin());
//         wall < walls.walls.end();
//         ++wall)
//    {
//        _haveToInspect.push_back(Wall(*wall));
//    }
}

bool callbackMarkTarget(ohm_exploration::MarkTarget::Request& req, ohm_exploration::MarkTarget::Response& res)
{
    for (std::vector<Target>::iterator target(_targets.begin()); target < _targets.end(); ++target)
    {
        if (req.id == target->id())
        {
            target->setInspected(true);
            return true;
        }
    }

    return false;
}

bool callbackGetTarget(ohm_exploration::GetTarget::Request& req, ohm_exploration::GetTarget::Response& res)
{
    if (req.id < 0)
    {
        if (!_haveToInspect.size())
        {
            ROS_ERROR("No walls in the stack!");
            return false;
        }

        _targets.push_back(_haveToInspect.front());
        _haveToInspect.pop_front();
        res.poses.resize(_targets.back().poses().size());
        res.id = _targets.back().id();

        for (unsigned int i = 0; i < _targets.back().poses().size(); ++i)
            res.poses[i] = _targets.back().poses()[i].toRos();

        return true;
    }

    for (std::vector<Target>::const_iterator target(_targets.begin()); target < _targets.end(); ++target)
    {
        if (req.id == target->id())
        {
            res.poses.resize(target->poses().size());

            for (unsigned int i = 0; i < target->poses().size(); ++i)
                res.poses[i] = target->poses()[i].toRos();

            return true;
        }
    }

    ROS_ERROR("Wall id not found.");
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "target_stack");
    std::string topic;
    ros::NodeHandle para("~");
    ros::NodeHandle nh;
    _listener = new tf::TransformListener;

    para.param<std::string>("topic_walls", topic, "exploration/walls");
    ros::Subscriber subWalls(nh.subscribe(topic, 2, callbackWalls));
    para.param<std::string>("service_get_target", topic, "exploration/get_target");
    ros::ServiceServer srvGetTarget(nh.advertiseService(topic, callbackGetTarget));
    para.param<std::string>("service_mark_target", topic, "exploration/mark_target");
    ros::ServiceServer srvMarkTarget(nh.advertiseService(topic, callbackMarkTarget));
    para.param<std::string>("service_plan_paths", topic, "path_plan/plan_paths");
    _srvPlanPaths = nh.serviceClient<ohm_path_plan::PlanPaths>(topic);
    para.param<std::string>("tf_source", _tfSource, "map");
    para.param<std::string>("tf_target", _tfTarget, "base_footprint");

    ros::spin();

    delete _listener;
}
