#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <ohm_autonomy/WallArray.h>
#include <ohm_autonomy/MarkTarget.h>
#include <ohm_autonomy/GetTarget.h>
#include <ohm_path_plan/PlanPaths.h>
#include <nav_msgs/GetMap.h>

#include <list>
#include <cstdlib>
#include <ctime>

#include "Wall.h"
#include "Target.h"
#include "TargetFactory.h"
#include "PartitionGrid.h"

std::vector<Target*> _targets;
ros::ServiceClient _srvPlanPaths;
ros::Publisher _pubGridMarker;
tf::TransformListener* _listener = 0;
std::string _tfSource, _tfTarget;
PartitionGrid* _grid = 0;

/*
Target takeClosestTargetFromList(std::list<Target>& targets, const Pose& origin)
{
    if (!targets.size())
        return Target();

    ohm_path_plan::PlanPaths paths;
    paths.request.origin = origin.toRos();

    for (std::list<Target>::const_iterator target(targets.begin()); target != targets.end(); ++target)
        paths.request.targets.push_back(target->pose().toRos());


    if (!_srvPlanPaths.call(paths))
    {
        ROS_ERROR("Cannot call the path plan node.");
        return Target();
    }

    if (paths.response.lengths.size() != targets.size())
    {
        ROS_ERROR("Not enough distances received from the path plan node.");
        return Target();
    }

    std::vector<double>::const_iterator distance(paths.response.lengths.begin());
    for (std::list<Target>::iterator target(targets.begin()); target != targets.end(); ++target, ++distance)
        target->setDistance(*distance < 0 ? std::numeric_limits<float>::max() : *distance);

    targets.sort();

    for (std::list<Target>::const_iterator target(targets.begin()); target != targets.end(); ++target)
	ROS_INFO("Target %i is %f m away.", target->id(), target->distance());

    Target target(targets.front());
    targets.pop_front();
    return target;
}
*/

void estimateDistancesFromOrigin(std::vector<Target*>& targets)
{
    if (!targets.size())
        return;

    ohm_path_plan::PlanPaths paths;
    paths.request.origin.position.x = 0.0;
    paths.request.origin.position.y = 0.0;
    paths.request.origin.position.z = 0.0;

    for (std::vector<Target*>::const_iterator target(targets.begin()); target < targets.end(); ++target)
        paths.request.targets.push_back((**target).pose().toRos());


    if (!_srvPlanPaths.call(paths))
    {
        ROS_ERROR("Cannot call the path plan node.");
        return;
    }
    if (paths.response.lengths.size() != targets.size())
    {
        ROS_ERROR("Not enough distances received from the path plan node.");
        return;
    }


    std::vector<double>::const_iterator distance(paths.response.lengths.begin());
    for (std::vector<Target*>::iterator target(targets.begin()); target < targets.end(); ++target, ++distance)
        (**target).setDistanceFromOrigin(*distance < 0 ? std::numeric_limits<float>::max() : *distance);
}

void estimateDistances(void)
{
    /* Get the current robot pose. */
    ROS_INFO("Get robot pose.");
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


    /* Estiamte all distances off the targets to the current robot pose. */
    const Pose robot(Eigen::Vector3f(transform.getOrigin().x(),
                                     transform.getOrigin().y(),
                                     transform.getOrigin().z()),
                     Eigen::Vector3f(1.0f, 0.0f, 0.0f));

    _grid->selected()->estimateDistances(_srvPlanPaths, robot);
}

void callbackWalls(const ohm_autonomy::WallArray& msg)
{
    TargetFactory factory;
    std::vector<Wall> walls;

    for (std::vector<ohm_autonomy::Wall>::const_iterator wall(msg.walls.begin()); wall < msg.walls.end(); ++wall)
        walls.push_back(Wall(*wall));


    factory.create(walls);
    estimateDistancesFromOrigin(factory.targets());
    _grid->insert(factory.targets());


//    Target target(takeClosestTargetFromList(targets, origin));
//
//    while (target.valid())
//    {
//        _haveToInspect.push_back(target);
//	ROS_INFO_STREAM("Take closest with position: " << target.pose().position);
//        target = takeClosestTargetFromList(targets, target.pose());
//    }
}

bool callbackMarkTarget(ohm_autonomy::MarkTarget::Request& req, ohm_autonomy::MarkTarget::Response& res)
{
    for (std::vector<Target*>::iterator target(_targets.begin()); target < _targets.end(); ++target)
    {
        if (req.id == (**target).id())
        {
            (**target).setInspected(true);
            return true;
        }
    }

    return false;
}

bool callbackGetTarget(ohm_autonomy::GetTarget::Request& req, ohm_autonomy::GetTarget::Response& res)
{
    if (req.id < 0)
    {
        _grid->switchToNextPartition();
        Partition* partition(_grid->selected());

        if (!partition)
        {
            ROS_ERROR("No valid partition found.");
            return false;
        }

        estimateDistances();

        if (!partition->numValidTargets())
        {
            ROS_ERROR("No target in the stack!");
            return false;
        }

        ROS_INFO("num valid targets = %d", partition->numValidTargets());
        Target* target = partition->target();

        res.pose = target->pose().toRos();
        res.id   = target->id();

        return true;
    }

    for (std::vector<Target*>::const_iterator target(_targets.begin()); target < _targets.end(); ++target)
    {
        if (req.id == (**target).id())
        {
            res.pose = (**target).pose().toRos();
            return true;
        }
    }

    ROS_ERROR("Target id not found.");
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


    /* Get a map and then create the target grid. */
    para.param<std::string>("service_get_map", topic, "map");
    ros::ServiceClient srvMap(nh.serviceClient<nav_msgs::GetMap>(topic));
    _pubGridMarker = nh.advertise<visualization_msgs::MarkerArray>("exploration/target_grid", 2);
    ros::service::waitForService(topic);

    nav_msgs::GetMap service;

    if (!srvMap.call(service))
    {
        ROS_ERROR("target_stack: can not call the service get map.");
        return 1;
    }

    _grid = new PartitionGrid(service.response.map, 1.2f);


    ::sleep(5);

    ros::spin();

//    ros::Rate rate(10);
//    srand(time(0));
//    while (ros::ok())
//    {
//        _pubGridMarker.publish(grid.getMarkerMsg());
//        ros::spinOnce();
//        rate.sleep();
//    }

    delete _listener;
    delete _grid;
}
