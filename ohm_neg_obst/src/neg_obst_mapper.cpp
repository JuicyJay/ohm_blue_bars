/*
 * neg_obst_mapper.cpp
 *
 *  Created on: Oct 29, 2017
 *      Author: phil
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include "ohm_neg_obst/ReconfigureHeightMapperConfig.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>

#include <string>

void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
void callbackDynamicReconfigure(ohm_neg_obst::ReconfigureHeightMapperConfig& config, uint32_t level);
void resetMap(nav_msgs::OccupancyGrid* const map, const double width, const double height, const double res);
bool freeFootprint(nav_msgs::OccupancyGrid* const map, const unsigned int width, const unsigned int height, const unsigned int offsetX, const unsigned int offsetY);

static ros::Publisher _pubMarkers;
static ros::Publisher _pubMap;
static nav_msgs::OccupancyGrid _negMap;

static double       _mapHeight   = 0.0;
static double       _mapWidth    = 0.0;
static double       _mapRes      = 0.0;
static bool         _growing     = true;
static unsigned int _nGrow       = 0;
static unsigned int _threshEmpty = 0;
static bool         _initialized = false;
static std::string  _tfFrameMap;

#define HEIGHT 20.0
#define WIDTH 20.0
#define RES 0.05
#define GROWING
#define N_GROW 1
#define N_EMPTY_THRESH 1

int main(int argc, char** argv)
{
  ros::init(argc, argv, "neg_obst_mapper");
  std::string topicCloudIn;
  std::string topicMarkers;
  std::string topicMap;

  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");
  prvNh.param<std::string>("topic_cloud_in", topicCloudIn, "outlier_filter/output");
  prvNh.param<std::string>("topic_markers", topicMarkers, "clusters");
  prvNh.param<std::string>("topic_map", topicMap, "neg_map");
  prvNh.param<std::string>("tf_frame_map", _tfFrameMap, "base_link");
  ros::Subscriber subsCloud = nh.subscribe(topicCloudIn, 1, callBackCloud);
  _pubMap = nh.advertise<nav_msgs::OccupancyGrid>(topicMap, 1);
  resetMap(&_negMap, WIDTH, HEIGHT, RES);
//  _negMap.header.frame_id = "base_link";
//  _negMap.info.resolution = RES;
//  _negMap.info.height = std::floor(HEIGHT / _negMap.info.resolution);
//  _negMap.info.width  = std::floor(WIDTH  / _negMap.info.resolution);
//  _negMap.info.origin.position.x = -HEIGHT / 2.0;
//  _negMap.info.origin.position.y = -WIDTH / 2.0;


  ros::spin();
}

void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  if(!_initialized)
  {
    std::cout << __PRETTY_FUNCTION__ << "not initialized" << std::endl;
    return;
  }
  _negMap.data.clear();
  _negMap.data.resize(_negMap.info.width * _negMap.info.height, -1);
  for(auto& iter : cloud.points)
  {
     //std::cout << __PRETTY_FUNCTION__ << " px py " << iter.x << " " << iter.y << std::endl;
    const unsigned int xIdx = static_cast<unsigned int>(std::floor((iter.x - _negMap.info.origin.position.x)  / _negMap.info.resolution));
    const unsigned int yIdx = static_cast<unsigned int>(std::floor((iter.y - _negMap.info.origin.position.y) / _negMap.info.resolution));
   // std::cout << __PRETTY_FUNCTION__ << "x y" << xIdx << " " << yIdx << std::endl;
    if((xIdx > _negMap.info.width) || (yIdx > _negMap.info.height))
      continue;
    _negMap.data[yIdx * _negMap.info.width + xIdx] = 0;
  }

  if(_growing)
  {
    std::vector<bool> mask(_negMap.info.height * _negMap.info.width, false);
    for(unsigned int row = _nGrow; row < _negMap.info.height - _nGrow; row++)
      for(unsigned int col = _nGrow; col < _negMap.info.width - _nGrow; col++)
      {
        if((_negMap.data[row * _negMap.info.width + col] >= 0) && !mask[row * _negMap.info.width + col])
        {

          //std::cout << __PRETTY_FUNCTION__ << " mongo" << std::endl;
          for(unsigned int subRow = row - _nGrow; subRow <= row + _nGrow; subRow++)
            for(unsigned int subCol = col - _nGrow; subCol <= col + _nGrow; subCol++)
            {
              _negMap.data[subRow * _negMap.info.width + subCol] = _negMap.data[row * _negMap.info.width + col];
              mask[subRow * _negMap.info.width + subCol] = true;
            }
        }
      }
  }
  freeFootprint(&_negMap, 20, 20, 0, 0);
  for(unsigned int row = 1; row < _negMap.info.height - 1; row++)
    for(unsigned int col = 1; col < _negMap.info.width - 1; col++)
    {
      if(_negMap.data[row * _negMap.info.width + col] != 0)
        continue;
      unsigned int nNeighBoursEmpty = 0;
      for(unsigned int subRow = row - 1; subRow <= row + 1; subRow++)
        for(unsigned int subCol = col - 1; subCol <= col + 1; subCol++)
        {
          if(_negMap.data[subRow * _negMap.info.width + subCol] == -1)
            nNeighBoursEmpty++;
        }
      if(nNeighBoursEmpty > _threshEmpty)
        _negMap.data[row * _negMap.info.width + col] = 100;
    }

  _pubMap.publish(_negMap);
}

void callbackDynamicReconfigure(ohm_neg_obst::ReconfigureHeightMapperConfig& config, uint32_t level)
{
  std::cout << __PRETTY_FUNCTION__ << "fdf" << std::endl;
  _mapHeight   = config.map_height;
  _mapWidth    = config.map_width;
  _mapRes      = config.map_res;
  _growing     = config.growing;
  _nGrow       = config.n_grow;
  _threshEmpty = config.thresh_empty;
  resetMap(&_negMap, _mapWidth, _mapHeight, _mapRes);

}

void resetMap(nav_msgs::OccupancyGrid* const map, const double width, const double height, const double res)
{
  map->header.frame_id = _tfFrameMap;
  map->info.resolution = res;
  map->info.height = std::floor(height / map->info.resolution);
  map->info.width  = std::floor(width  / map->info.resolution);
  map->info.origin.position.x = -height / 2.0;
  map->info.origin.position.y = -width  / 2.0;
  if(!_initialized)
     _initialized = true;
}

bool freeFootprint(nav_msgs::OccupancyGrid* const map, const unsigned int width, const unsigned int height, const unsigned int offsetX, const unsigned int offsetY)
{
  for(unsigned int row = map->info.height / 2 - width / 2; row < map->info.height / 2 + width / 2; row++)
    for(unsigned int col = map->info.width / 2 - height / 2; col < map->info.width / 2 + height / 2; col++)
    {
      map->data[row * map->info.width + col] = 0;
    }
  return true;
}
