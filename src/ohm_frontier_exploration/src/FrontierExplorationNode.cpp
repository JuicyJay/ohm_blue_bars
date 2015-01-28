/*
 * FrontierExplorationNode.cpp
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#include "FrontierExplorationNode.h"

#include "geometry_msgs/PoseArray.h"

#include "MapSubsampler.h"

#include "FrontierController.h"


#include <visualization_msgs/Marker.h>


namespace autonohm {

// initialization of singleton pattern
FrontierExplorationNode* FrontierExplorationNode::_instance = 0;


FrontierExplorationNode* FrontierExplorationNode::getInstance(void)
{
   if(!_instance) _instance = new FrontierExplorationNode;
   return _instance;
}

void FrontierExplorationNode::run(void)
{
   while(ros::ok())
   {
      ros::spinOnce();
   }
}


FrontierExplorationNode::FrontierExplorationNode(void) :
      _frontierFinder(NULL)
{
   ros::NodeHandle private_nh("~");

   frontier::FinderConfig config;
   private_nh.param<double>("robot_radius",               config.robot_radius,               0.6);
   private_nh.param<double>("min_dist_between_frontiers", config.min_dist_between_frontiers, 1.0);
   private_nh.param<double>("max_search_radius",          config.max_search_radius,          10.0);
   _frontierFinder = new frontier::Finder(config);

   std::string map_topic;
   std::string frontier_topic;
   private_nh.param("map_topic",       map_topic,      std::string("/map"));
   private_nh.param("frontier_topic",  frontier_topic, std::string("frontiers"));

   // Publishers
   _frontier_pub      = _nh.advertise<geometry_msgs::PoseArray>(frontier_topic,  1);

   // Subscriber
   _map_sub           = _nh.subscribe(map_topic, 1, &FrontierExplorationNode::mapCallback, this);
   _sub_map_pub       = _nh.advertise<nav_msgs::OccupancyGrid>("sub_map",  1);
   _frontier_grid_pub = _nh.advertise<nav_msgs::GridCells>("frontier_grid", 1);
//   _maker_pub         = _nh.advertise<visualization_msgs::Marker>("frontier_marker", 1);


//   std::cout << config << std::endl;
//   ROS_DEBUG_STREAM("Starting frontier exploration node with following parameters: "
//                     << std::endl << config << std::endl);


   _viz.setNodeHandle(_nh);

   // start node
   this->run();
}


FrontierExplorationNode::~FrontierExplorationNode(void)
{
   if(_instance) {
      delete _instance;
      _instance = 0;
   }
}


void FrontierExplorationNode::findFrontiers(void)
{
   if(_frontierFinder->isInitialized())
   {
      _frontierFinder->calculateFrontiers();
      _frontiers = _frontierFinder->getFrontiers();

      this->publishFrontiers();
   }
   else
   {
      std::cout << "not initialized" << std::endl;
      ROS_DEBUG_STREAM("FrontierFinder not initialized");
   }
}


void FrontierExplorationNode::publishFrontiers(void)
{
   // publish frontiers for rviz and further calculation
   static unsigned int seq = 0;
   geometry_msgs::PoseArray frontierMarkers;
   frontierMarkers.header.stamp    = ros::Time::now();
   frontierMarkers.header.seq      = seq++;
   frontierMarkers.header.frame_id = "/map";

   // fill message
   for(std::vector<Frontier>::iterator it=_frontiers.begin() ; it != _frontiers.end() ; ++it)
      frontierMarkers.poses.push_back(*it);

   // publish message
   _frontier_pub.publish(frontierMarkers);

   _frontier_grid_pub.publish(_frontierFinder->getFrontierLayer());

//   // look for best frontier
   FrontierController fController;
   fController.setWeightedFrontiers(_frontierFinder->getWeightedFrontiers());
   fController.findBestFrontier();

   _viz.setFrontiers(_frontiers);
   _viz.setBestFrontier(fController.getBestFrontier());
   std::vector<WeightedFrontier> wf = fController.getWeightedFrontiers();
   _viz.setWeightedFrontiers(wf);
   _viz.publish();

//   std::cout << "bestFrontier: " << best.position.x << ", " << best.position.y << std::endl;

//   visualization_msgs::Marker frontiers;
//   frontiers.header.frame_id = "/map";
//   frontiers.ns              = "frontiers";
//   frontiers.pose            = fController.getBestFrontier();
//
//   frontiers.type            = visualization_msgs::Marker::CYLINDER;
//   frontiers.scale.x         = 1.0;
//   frontiers.scale.y         = 1.0;
//   frontiers.scale.z         = 1.0;
//   frontiers.color.r         = 1.0;
//   frontiers.color.a         = 1.0;
//
//   _maker_pub.publish(frontiers);
}


void FrontierExplorationNode::mapCallback(const nav_msgs::OccupancyGrid& map)
{
   ROS_DEBUG_STREAM("received new map. ");
   _frontierFinder->setMap(map);

   this->findFrontiers();
   this->publishFrontiers();
}

void FrontierExplorationNode::publishMarkers(void)
{

}

} /* namespace autonohm */


/*
 * Main program
 */
int main(int argc,char **argv)
{
   ros::init(argc, argv, "frontier_exploration_node");
   autonohm::FrontierExplorationNode::getInstance()->run();
}

