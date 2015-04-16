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
#include <nav_msgs/GetMap.h>

#include "FindWall.h"
#include "Rect.h"

#include <ohm_autonomy/WallArray.h>
#include <ohm_common/MapRoi.h>

FindWall _wallFinder;
std::vector<Wall> _walls;
ros::Publisher _pubWallMarkers;
ros::Publisher _pubWalls;
ros::Subscriber _subRoi;
ros::ServiceClient _srvGetMap;
bool _initialized = false;
Rect _roi;
float _resolution = 1.0f;
float _originMapX = 0.0f;
float _originMapY = 0.0f;

bool callbackTrigger(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    nav_msgs::GetMap service;

    if (!_srvGetMap.call(service))
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": can not call the service get map. Will not search walls.");
        return false;
    }

    if (!_initialized)
    {
        _wallFinder.setMap(service.response.map);
        _initialized = true;
	_resolution = service.response.map.info.resolution;
	_originMapX = service.response.map.info.origin.position.x;
	_originMapY = service.response.map.info.origin.position.y;
    }


    /* Update the map in wallFinder and then looking for walls. */
    std::vector<Wall> walls;
    _wallFinder.updateMap(service.response.map, _roi);
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

    return true;
}

void callbackRoi(const ohm_common::MapRoi& msg)
{
  return;
  if (!_initialized)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": no map received yet. Can not set the roi.");
      return;
    }

  _roi.setX((msg.origin.x - _originMapX) / _resolution);
  _roi.setY((msg.origin.y - _originMapY) / _resolution);
  _roi.setWidth(msg.width / _resolution);
  _roi.setHeight(msg.height / _resolution);

  ROS_INFO("set roi = (%d, %d, %d, %d)", _roi.x(), _roi.y(), _roi.width(), _roi.height());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wall_finder");
    ros::NodeHandle para("~");
    ros::NodeHandle nh;
    std::string topic;

    para.param<std::string>("service_trigger", topic, "exploration/wall_finder/trigger");
    ros::ServiceServer srvTrigger(nh.advertiseService(topic, callbackTrigger));
    para.param<std::string>("service_get_map", topic, "map");
    _srvGetMap = nh.serviceClient<nav_msgs::GetMap>(topic);

    _pubWallMarkers = nh.advertise<visualization_msgs::MarkerArray>("exploration/wall_markers", 2);
    _pubWalls = nh.advertise<ohm_autonomy::WallArray>("exploration/walls", 2);
    _subRoi = nh.subscribe("exploration/set_roi", 2, callbackRoi);

    ros::spin();
}
