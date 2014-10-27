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
    ros::NodeHandle nh;
    ros::Subscriber subMap(nh.subscribe("map", 1, callbackMap));
    ros::ServiceServer srvTrigger(nh.advertiseService("exploration/wall_finder/trigger", callbackTrigger));
    _pubWallMarkers = nh.advertise<visualization_msgs::MarkerArray>("exploration/wall_markers", 2);

    ros::spin();
}
