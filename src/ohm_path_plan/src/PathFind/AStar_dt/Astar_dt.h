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
#include "../PathFind_base/PathFind_base.h"
#include <queue>
#include <cmath>
//#include <Eigen/Dense>

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
                      astar_f_type factor_dist )
   {
      _cost_short_step = cost_short_step;
      _cost_long_step  = cost_long_step;
      _factor_dist     = factor_dist;
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

      uint8_t costMapVal = _costMapStack.at("dt")->getMapData()[this->pixelToIdx(curr,
                                                                this->getGridMap()->getWidth())];
      astar_f_type scale = 0;
      if(costMapVal != 0)
      {
         scale = (astar_f_type)costMapVal / (astar_f_type)255;
         //scale =  1 - scale;//scale *= 0.0001;
         //todo continue here
      }
      //printf("scale: %f\n", scale);

      return tmp * (scale + 1);
   }

   void getChildNodes(std::vector<Astar_child>& vec, Pixel node);

   //astar_f_type updateFx(Astar_child currNode);

   void tracePath(Astar_node* node);

   inline astar_f_type getGx(Astar_node* currNode, Astar_child child)
   {
      astar_f_type tmp = currNode->g + (child.isDiagonal ? _cost_long_step : _cost_short_step);
      //uint8_t costMapVal = _costMapStack.at("dt")->getMapData()[this->pixelToIdx(child.pixel,
      //                                                                           this->getGridMap()->getWidth())];
      //value range: 0..1

      //scale *= 0.1;
      //tmp *=
      return tmp;// * (scale + 1);
   }

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

};

} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_ASTAR_ASTAR_H_ */
