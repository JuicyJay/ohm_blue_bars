#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "FindWall.h"

FindWall _wallFinder;
std::vector<Wall> _walls;

void callbackMap(const nav_msgs::OccupancyGrid& map)
{
    _wallFinder.setMap(map);
    _wallFinder.search(_walls);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_finder");
    ros::NodeHandle nh;
    ros::Subscriber subMap(nh.subscribe("map", 1, callbackMap));

    ros::spin();
}
