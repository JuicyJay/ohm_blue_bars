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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/point_types.h>

#include <string>

void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

static ros::Publisher _pubMarkers;
static ros::Publisher _pubMap;
static nav_msgs::OccupancyGrid _negMap;

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
  ros::Subscriber subsCloud = nh.subscribe(topicCloudIn, 1, callBackCloud);
  _pubMap = nh.advertise<nav_msgs::OccupancyGrid>(topicMap, 1);
  _negMap.header.frame_id = "base_link";
  _negMap.info.resolution = RES;
  _negMap.info.height = std::floor(HEIGHT / _negMap.info.resolution);
  _negMap.info.width  = std::floor(WIDTH  / _negMap.info.resolution);
  _negMap.info.origin.position.x = -HEIGHT / 2.0;
  _negMap.info.origin.position.y = -WIDTH / 2.0;


  ros::spin();
}

void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  _negMap.data.clear();
  _negMap.data.resize(_negMap.info.width * _negMap.info.height, -1);
  for(auto& iter : cloud.points)
  {
    // std::cout << __PRETTY_FUNCTION__ << " px py " << iter.x << " " << iter.y << std::endl;
    const unsigned int xIdx = static_cast<unsigned int>(std::floor((iter.x + HEIGHT / 2.0) / _negMap.info.resolution));
    const unsigned int yIdx = static_cast<unsigned int>(std::floor((iter.y + WIDTH / 2.0 )/ _negMap.info.resolution));
    //  std::cout << __PRETTY_FUNCTION__ << "x y" << xIdx << " " << yIdx << std::endl;
    if((xIdx > _negMap.info.width) || (yIdx > _negMap.info.height))
      continue;
    _negMap.data[yIdx * _negMap.info.width + xIdx] = 0;
  }
#ifdef GROWING
  std::vector<bool> mask(_negMap.info.height * _negMap.info.width, false);
  for(unsigned int row = N_GROW; row < _negMap.info.height - N_GROW; row++)
    for(unsigned int col = N_GROW; col < _negMap.info.width - N_GROW; col++)
    {
      if((_negMap.data[row * _negMap.info.width + col] >= 0) && !mask[row * _negMap.info.width + col])
      {

        //std::cout << __PRETTY_FUNCTION__ << " mongo" << std::endl;
        for(unsigned int subRow = row - N_GROW; subRow <= row + N_GROW; subRow++)
          for(unsigned int subCol = col - N_GROW; subCol <= col + N_GROW; subCol++)
          {
            _negMap.data[subRow * _negMap.info.width + subCol] = _negMap.data[row * _negMap.info.width + col];
            mask[subRow * _negMap.info.width + subCol] = true;
          }
      }
    }
#endif
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
      if(nNeighBoursEmpty > N_EMPTY_THRESH)
        _negMap.data[row * _negMap.info.width + col] = 100;
    }
  _pubMap.publish(_negMap);
}
