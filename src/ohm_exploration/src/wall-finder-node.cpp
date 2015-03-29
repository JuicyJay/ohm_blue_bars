/***********************************************************************************************************
 * wall-finder-node.cpp
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ***********************************************************************************************************/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>

#include "FindWall.h"

#include <ohm_autonomy/WallArray.h>

FindWall _wallFinder;
std::vector<Wall> _walls;
ros::Publisher _pubWallMarkers;
ros::Publisher _pubWalls;
bool _armed = false;
bool _initialized = false;

bool callbackTrigger(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    _armed = true;

    return true;
}

void callbackMap(const nav_msgs::OccupancyGrid& map)
{
    if (!_initialized)
    {
        _wallFinder.setMap(map);
        _initialized = true;
        return;
    }

    if (!_armed)
        return;


    /* Update the map in wallFinder and then looking for walls. */
    std::vector<Wall> walls;
    _armed = false;
    _wallFinder.updateMap(map);
    _wallFinder.search(walls);
    _walls.insert(_walls.end(), walls.begin(), walls.end());


    /* Send found walls. */
    ohm_autonomy::WallArray msgWalls;

    for (std::vector<Wall>::const_iterator wall(walls.begin()); wall < walls.end(); ++wall)
        msgWalls.walls.push_back(wall->getWallMessage());

    _pubWalls.publish(msgWalls);


    /* Publish markers to show it in rviz. */
    if (_pubWallMarkers.getNumSubscribers())
    {
        visualization_msgs::MarkerArray msg;

        for (std::vector<Wall>::const_iterator wall(_walls.begin()); wall < _walls.end(); ++wall)
            msg.markers.push_back(wall->getMarkerMessage());

        _pubWallMarkers.publish(msg);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_finder");
    ros::NodeHandle para("~");
    ros::NodeHandle nh;
    std::string topic;

    para.param<std::string>("service_trigger", topic, "exploration/wall_finder/trigger");
    ros::ServiceServer srvTrigger(nh.advertiseService(topic, callbackTrigger));
    ros::Subscriber subMap(nh.subscribe("map", 1, callbackMap));
    _pubWallMarkers = nh.advertise<visualization_msgs::MarkerArray>("exploration/wall_markers", 2);
    _pubWalls = nh.advertise<ohm_autonomy::WallArray>("exploration/walls", 2);

    ros::spin();
}
