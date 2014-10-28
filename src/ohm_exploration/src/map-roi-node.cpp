#include <ros/ros.h>

#include "Map.h"

const Rect _roi(12, 12, 200, 200);
ros::Publisher _pubMap;

void sendMap(const Map& map)
{
    static uint32_t seq = 0;
    nav_msgs::OccupancyGrid msg;

    msg.header.seq = ++seq;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";

    msg.info.resolution = map.resolution();
    msg.info.width = map.width();
    msg.info.height = map.height();
    msg.info.origin.position.x = map.origin().x();
    msg.info.origin.position.y = map.origin().y();

    msg.data.resize(map.width() * map.height());

    for (unsigned int y = 0; y < map.height(); ++y)
        for (unsigned int x = 0; x < map.width(); ++x)
            msg.data[y * map.width() + x] = map(x, y);

    _pubMap.publish(msg);
}

void callbackMap(const nav_msgs::OccupancyGrid& map)
{
    nav_msgs::OccupancyGrid dummy = map;
    Map bigMap(dummy);
    Map smallMap(bigMap, _roi);
    sendMap(smallMap);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_roi");
    ros::NodeHandle nh;

    ros::Subscriber subMap(nh.subscribe("map", 1, callbackMap));
    _pubMap = nh.advertise<nav_msgs::OccupancyGrid>("map_roi", 2);

    ros::spin();
}
