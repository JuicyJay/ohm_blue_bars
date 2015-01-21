/*
 * PathFind_base.h
 *
 *  Created on: 15.12.2014
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_PATHFIND_BASE_PATHFIND_BASE_H_
#define ALGORITHM_PATHFIND_PATHFIND_BASE_PATHFIND_BASE_H_

#include <iostream>
#include <vector>
#include <map>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <stdint.h>
#include "../Map/GridMap.h"

namespace apps
{

class PathFind_base
{
public:
   PathFind_base(GridMap* map);
   virtual ~PathFind_base();

   void setGridMap(GridMap* map);
   GridMap* getGridMap() const;

   /**
    * @brief binarize the map
    *
    * @param min  lower limit
    * @param max  upper limit
    * @param vel_in  value of resulting map, values between min and max
    * @param vel_out value of resulting map, values outside min and max
    */
   //void binarize(uint8_t min, uint8_t max, uint8_t vel_in, uint8_t vel_out);

   /**
    * @brief inflates mappoints with value "val"
    *
    * @param val     value whitch points got inflated
    * @param offset  inflrate range;
    */
  //void inflate(uint8_t val, double offset);

   //void truncate(/*todo*/) {} //todo cutt of map stuff whitch is not needd

   /**
    * @brief addes Costmap... if key already exists does update existing
    *
    * @param key
    * @param costmap
    * @return true if succes or false if not succes(params does not fit)
    */
   bool addCostmap(std::string key, GridMap* costmap);

   /**
    *
    * @param key
    * @return return true if element was exsiting, false if not
    */
   bool removeCostmap(std::string key);

   GridMap* getCostmap(std::string key);
   /**
    * @brief removes costmaps from map... does not free memory
    */
   void resetCostmaps();

   //void setRobotRadius(const double rad) { _robot_radius = rad; }
   //double getRobotRadius() const { return _robot_radius; }

   std::vector<Point2D> computePathPoint(Point2D start, Point2D end);
   virtual std::vector<Pixel> computePath(Pixel start, Pixel end) = 0;

private: //functions

protected:
   GridMap* _map;
   //double _robot_radius;

   std::map<std::string,GridMap*> _costMapStack;


};

} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_PATHFIND_BASE_PATHFIND_BASE_H_ */
