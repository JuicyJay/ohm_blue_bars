#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

#include "FindWall.h"

FindWall _wallFinder;
std::vector<Wall> _walls;
ros::Publisher _pubWallMarkers;
bool _armed = false;

bool callbackTrigger(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    _armed = true;

    return true;
}

void callbackMap(const nav_msgs::OccupancyGrid& map)
{
    if (!_armed)
        return;

    _armed = false;
    _wallFinder.setMap(map);
    _wallFinder.search(_walls);

    visualization_msgs::MarkerArray msg;

    for (std::vector<Wall>::const_iterator wall(_walls.begin()); wall < _walls.end(); ++wall)
        msg.markers.push_back(wall->getMarkerMessage());

    _pubWallMarkers.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_finder");
    ros::NodeHandle para("~");
    ros::NodeHandle nh;
    std::string topic;

    para.param<std::string>("topic_map", topic, "/map");
    ros::Subscriber subMap(nh.subscribe(topic, 1, callbackMap));
    para.param<std::string>("service_trigger", topic, "exploration/wall_finder/trigger");
    ros::ServiceServer srvTrigger(nh.advertiseService(topic, callbackTrigger));
    para.param<std::string>("topic_markers", topic, "exploration/wall_markers");
    _pubWallMarkers = nh.advertise<visualization_msgs::MarkerArray>(topic, 2);

    ros::spin();
}
