/*
 * FrontierFinder.h
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#ifndef OHM_FRONTIER_EXPLORATION_SRC_FRONTIERFINDER_H_
#define OHM_FRONTIER_EXPLORATION_SRC_FRONTIERFINDER_H_

#include "nav_msgs/OccupancyGrid.h"
#include "Frontier.h"

#include <ostream>
/**
 * @namespace autonohm
 */
namespace autonohm {



/**
 * @struct  FrontierFinderConfig
 * @author  Christian Pfitzner
 * @date    2015-01-26
 *
 * @brief   Parameters for configuration of
 */
struct FrontierFinderConfig
{
   double robot_radius;                    //!< size of robot as minimum frontier size

   double min_dist_between_frontiers;      //!< minimum distance between two frontiers

   double max_search_radius;               //!< search radius around robot for frontier search can be used to save computation cost

};

//friend std::ostream& operator<<(std::ostream &output, const FrontierFinderConfig &c)
//{
//       output<< "p_robot_radius"             << c.robot_radius               << std::endl;
////                << "p_min_dist_frontiers"       << c.min_dist_between_frontiers << std::endl;
////                << "p_max_search_radius"        << c.max_search_radius          << std::endl;
//return output;
//}







/**
 * @class   FrontierFinder
 * @author  Christian Pfitzner
 * @date    2014-01-26
 *
 * @brief   Frontier based finding on publication from Yamauchi 1998
 */
class FrontierFinder
{
public:
   /**
    * Default constructor
    */
   FrontierFinder(void);
   /**
    * Constructor with config initialization
    * @param config
    */
   FrontierFinder(FrontierFinderConfig config);
   /**
    * Default destructor
    */
   virtual ~FrontierFinder(void);


   // SETTERS
   /**
    * Function to set map for frontier estimation
    * @param map
    */
   void setMap(const nav_msgs::OccupancyGrid& map);
   /**
    * Function to set configuration
    * @param config
    */
   void setConfig(FrontierFinderConfig config);


   // GETTERS
   /**
    * Function to get frontiers
    * @return
    */
   std::vector<Frontier> getFrontiers(void);


   /**
    * Function to start calculation
    */
   void calculateFrontiers(void);
   /**
    * Function to check if frontier search is initialized
    * @return
    */
   bool isInitialized(void)   { return _initialized; }

private:
   bool                       _initialized;

   nav_msgs::OccupancyGrid    _map;                //!< map for exploration
   FrontierFinderConfig       _config;             //!< config for exploration
   std::vector<Frontier>      _frontiers;          //!< container for found frontiers
};



} /* namespace autonohm */

#endif /* OHM_FRONTIER_EXPLORATION_SRC_FRONTIERFINDER_H_ */
