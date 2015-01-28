/*
 * FrontierExplorationNode.h
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_FRONTIEREXPLORATIONNODE_H_
#define OHM_FRONTIER_EXPLORATION_SRC_FRONTIEREXPLORATIONNODE_H_

#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "Frontier.h"
#include "FrontierFinder.h"

/**
 * @namespace autonohm
 */
namespace autonohm
{

/**
 *
 */
class FrontierExplorationNode
{
public:
   /**
    * Function to get instance of singleton
    * @return
    */
   static FrontierExplorationNode* getInstance(void);

   /**
    * Default destructor
    */
   virtual ~FrontierExplorationNode(void);

   /**
    * Function to spin ros node
    */
   void run(void);

private:
   /**
    * Private constructor for singleton pattern
    */
   FrontierExplorationNode(void);
   /**
    * Private copy constructor for singleton pattern
    * @param
    */
   FrontierExplorationNode(FrontierExplorationNode &) { }

   /**
    * Function to search for frontiers
    */
   void findFrontiers(void);
   /**
    * Function to publish frontiers
    */
   void publishFrontiers(void);

   // CALLBACK FUNCTIONS
   /**
    * Callback function for map
    * @param map
    */
   void mapCallback(const nav_msgs::OccupancyGrid& map);

   // MEMBERS
   static FrontierExplorationNode* _instance;

   ros::NodeHandle                  _nh;

   ros::Subscriber                  _map_sub;
   ros::Publisher                   _sub_map_pub;
   ros::Publisher                   _frontier_pub;

   std::vector<Frontier>            _frontiers;

   FrontierFinder*                  _frontierFinder;
};

} /* namespace autonohm */

#endif /* OHM_FRONTIER_EXPLORATION_SRC_FRONTIEREXPLORATIONNODE_H_ */