#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <list>

#include "Wall.h"
#include "Target.h"
#include "ohm_exploration/WallArray.h"
#include "ohm_exploration/MarkTarget.h"
#include "ohm_exploration/GetTarget.h"

std::list<Wall> _haveToInspect;
std::vector<Target> _targets;

void callbackWalls(const ohm_exploration::WallArray& walls)
{
    for (std::vector<ohm_exploration::Wall>::const_iterator wall(walls.walls.begin());
         wall < walls.walls.end();
         ++wall)
    {
        _haveToInspect.push_back(Wall(*wall));
    }
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

    para.param<std::string>("topic_walls", topic, "/exploration/walls");
    ros::Subscriber subWalls(nh.subscribe(topic, 2, callbackWalls));
    para.param<std::string>("service_get_target", topic, "/exploration/get_target");
    ros::ServiceServer srvGetTarget(nh.advertiseService(topic, callbackGetTarget));
    para.param<std::string>("service_mark_target", topic, "/exploration/mark_target");
    ros::ServiceServer srvMarkTarget(nh.advertiseService(topic, callbackMarkTarget));

    ros::spin();
}
