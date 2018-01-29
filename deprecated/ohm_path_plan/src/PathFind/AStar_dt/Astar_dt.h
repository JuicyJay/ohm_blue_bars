/*
 * Astar.h
 *
 *  Created on: 15.12.2014
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_ASTAR_ASTAR_H_
#define ALGORITHM_PATHFIND_ASTAR_ASTAR_H_

#include <iostream>
#include <stdio.h>

#include <queue>
#include <cmath>
//#include <Eigen/Dense>

#include "../PathFind_base/PathFind_base.h"
#include "../Map/MapOperations/MapOperations.h"

namespace apps
{


typedef float astar_f_type;


const unsigned int MAX_NUM_CHILD = 8;
//const astar_f_type COST_STEP_SHORT = 1;
//const astar_f_type COST_STEP_LONG = COST_STEP_SHORT * ::sqrt(2);//;COST_STEP_SHORT;// * 1.4 + 0.5;
//const astar_f_type FACTOR_DIST = 1;



struct Astar_node{
   astar_f_type f;
   astar_f_type g;
   unsigned int idx;
   bool isDiagonal;
   bool isOverritten;
   Pixel pixel;
   Astar_node* parent;
};

struct Astar_child{
   Pixel pixel;
   bool isDiagonal;

   Astar_child() : pixel(Pixel(0,0)) , isDiagonal(false) { }
   Astar_child(Pixel p, bool isD) : pixel(p) , isDiagonal(isD) { }
};

class Astar_node_comp {
public:
   bool operator() (const Astar_node* l, const Astar_node* r)
   {
      return l->f > r->f;
   }
};

/**
 * @class Astar
 *
 * @todo source out inflate and other map operations into map op class
 */
class Astar_dt: public PathFind_base
{
public:
   Astar_dt(GridMap* map);
   virtual ~Astar_dt();

   virtual std::vector<Pixel> computePath(Pixel start, Pixel end);

   /**
    * @brief sets wall_value for computing path. This value will interpreted as wall,
    *        default value = 255
    *
    * @todo move to base_path panner
    *
    * @param wall_value
    */
   void setWallValue(uint8_t wall_value)
   {
      _wall_value = wall_value;
   }

   void setAstarParam(astar_f_type cost_short_step,
                      astar_f_type cost_long_step,
                      astar_f_type factor_dist,
                      astar_f_type costmap_weight)
   {
      _cost_short_step = cost_short_step;
      _cost_long_step  = cost_long_step;
      _factor_dist     = factor_dist;
      _costmap_weight  = costmap_weight;
   }



private:

   inline unsigned int pixelToIdx(Pixel pix, unsigned int width)
   {
      return pix.y * width + pix.x;
   }

   inline Pixel idxToPixel(unsigned int idx, unsigned int width)
   {
      Pixel tmp;
      tmp.x = idx % width;
      tmp.y = idx / width;
      return tmp;
   }

   inline astar_f_type getHx(Pixel curr, Pixel end)
   {
      int x = end.x - curr.x;
      int y = end.y - curr.y;
      astar_f_type tmp = (::sqrt(x*x + y*y) * _factor_dist);

      astar_f_type scale = 0;

      scale = this->getCostmapScale(curr);

      //printf("scale: %f\n", scale);

      return tmp * (scale + 1);
   }


   inline astar_f_type getGx(Astar_node* currNode, Astar_child child)
   {
      astar_f_type tmp = currNode->g + (child.isDiagonal ? _cost_long_step : _cost_short_step);

      //astar_f_type scale = this->getCostmapScale(currNode->pixel);

      return tmp;// * (scale);
   }

   /**
    * @brief returs the merged scale from all costmaps
    *
    * If only one costmap sets a pixel as "wall" then the pixel is a wall.
    *
    * @todo Higher values from a cos map may should have an higher priority
    *
    * @return scalevalue [0..1], if wall -> [>2]
    */
   inline astar_f_type getCostmapScale(Pixel p)
   {
      astar_f_type scale = 0;

      //if no costmap... return 0
      if(!_costMapStack.size())
      {
         return 0;
      }

      //get values from all costmaps and merge....
      std::vector<uint8_t> mapValues;
      for(std::map<std::string,GridMap*>::iterator it = _costMapStack.begin(); it != _costMapStack.end(); ++it)
      {
         uint8_t val = it->second->getMapData()[MapOperations::pixelToIdx(p, it->second->getWidth())];
         if(val == 255)
         {//return wall detected (return >2)
            return 3.0;
         }
         mapValues.push_back(val);
      }

      //compute scale
      //todo get higher values higher priority... for now just average value is build

      for(unsigned int i = 0; i < mapValues.size(); ++i)
      {
         scale += (astar_f_type)mapValues[i] / (double)255;
      }

      scale /= mapValues.size();

      return scale * _costmap_weight;
   }

   void getChildNodes(std::vector<Astar_child>& vec, Pixel node);

   //astar_f_type updateFx(Astar_child currNode);

   void tracePath(Astar_node* node);



private:
   //std::priority_queue<unsigned int> _openList;
   std::vector<Pixel> _path;

   //astar_f_type _currentCosts;
   Pixel _st;
   Pixel _en;

   uint8_t _wall_value;

   astar_f_type _cost_short_step;
   astar_f_type _cost_long_step;
   astar_f_type _factor_dist;
   astar_f_type _costmap_weight;

};

} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_ASTAR_ASTAR_H_ */
