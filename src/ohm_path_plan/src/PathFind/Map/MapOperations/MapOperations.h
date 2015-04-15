/*
 * MapOperations.h
 *
 *  Created on: 29.12.2014
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_
#define ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_

#include "../GridMap.h"
#include <cmath>

namespace apps
{

class MapOperations
{
public:
   MapOperations() { }
   virtual ~MapOperations() { }
   static void binarize(GridMap* map, uint8_t min, uint8_t max, uint8_t vel_in, uint8_t vel_out);

   static void inflateRect(GridMap* map, uint8_t val_min, uint8_t val_max, double offset);
   static void distnaceTransformRect(GridMap* map, double offset, uint8_t wall_val);

   static void inflateCirc(GridMap* map, uint8_t val_min, uint8_t val_max, double offset);
   static void distnaceTransformCirc(GridMap* map, double offset, uint8_t wall_val);

   static void drawFilledCircle(GridMap* map, Point2D circ_center, double radius, uint8_t circ_value);


   static inline unsigned int pixelToIdx(Pixel pix, unsigned int width)
   {
      return pix.y * width + pix.x;
   }

   static inline Pixel idxToPixel(unsigned int idx, unsigned int width)
   {
      Pixel tmp;
      tmp.x = idx % width;
      tmp.y = idx / width;
      return tmp;
   }


private:
   static void drawCircle(uint8_t* data, unsigned int width, Pixel c, unsigned int radius, uint8_t value);


};

} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_MAP_MAPOPERATIONS_MAPOPERATIONS_H_ */
