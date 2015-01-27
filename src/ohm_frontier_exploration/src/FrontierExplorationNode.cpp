/*
 * FrontierExplorationNode.cpp
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#include "FrontierExplorationNode.h"

#include "geometry_msgs/PoseArray.h"

#include "MapSubsampler.h"





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

   FrontierFinderConfig config;
   private_nh.param<double>("robot_radius",               config.robot_radius,               0.6);
   private_nh.param<double>("min_dist_between_frontiers", config.min_dist_between_frontiers, 1.0);
   private_nh.param<double>("max_search_radius",          config.max_search_radius,          10.0);
   _frontierFinder = new FrontierFinder(config);

   std::string map_topic;
   std::string frontier_topic;
   private_nh.param("map_topic",       map_topic,      std::string("/map"));
   private_nh.param("frontier_topic",  frontier_topic, std::string("frontiers"));

   // Publishers
   _frontier_pub = _nh.advertise<geometry_msgs::PoseArray>(frontier_topic,  1);

   // Subscriber
   _map_sub      = _nh.subscribe(map_topic, 1, &FrontierExplorationNode::mapCallback, this);
   _sub_map_pub  = _nh.advertise<nav_msgs::OccupancyGrid>("sub_map",  1);



//   std::cout << config << std::endl;
//   ROS_DEBUG_STREAM("Starting frontier exploration node with following parameters: "
//                     << std::endl << config << std::endl);


   std::cout << "finished constructor" << std::endl;

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
}


void FrontierExplorationNode::mapCallback(const nav_msgs::OccupancyGrid& map)
{
//   MapSubsampler sampler;
//   sampler.setInput(map);
//   sampler.convert();
//   _sub_map_pub.publish(sampler.getSubsampledMap());


   ROS_DEBUG_STREAM("received new map. ");
   _frontierFinder->setMap(map);

   this->findFrontiers();
   this->publishFrontiers();
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

