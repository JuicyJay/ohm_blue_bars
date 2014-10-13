#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "FindWall.h"

FindWall _wallFinder;
std::vector<Wall> _walls;
ros::Publisher _pubWall;

void callbackMap(const nav_msgs::OccupancyGrid& map)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    _walls.clear();
    _wallFinder.setMap(map);
    _wallFinder.search(_walls);

    _pubWall.publish(_walls[0].getMarkerMessage());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_finder");
    ros::NodeHandle nh;
    ros::Subscriber subMap(nh.subscribe("map", 1, callbackMap));
    _pubWall = nh.advertise<visualization_msgs::Marker>("walls", 2);

    ros::spin();
}
