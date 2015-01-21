/*
 * MapOperations.h
 *
 *  Created on: 29.12.2014
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_
#define ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_

#include "../GridMap.h"

namespace apps
{

class MapOperations
{
public:
   MapOperations() { }
   virtual ~MapOperations() { }
   static void binarize(GridMap* map, uint8_t min, uint8_t max, uint8_t vel_in, uint8_t vel_out);
   static void inflate(GridMap* map, uint8_t val_min, uint8_t val_max, double offset);
   static void distnaceTransform(GridMap* map, double offset, uint8_t wall_val);

};

} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_ */
