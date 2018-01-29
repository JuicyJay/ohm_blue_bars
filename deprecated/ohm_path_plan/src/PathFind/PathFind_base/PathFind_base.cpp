/*
 * PathFind_base.cpp
 *
 *  Created on: 15.12.2014
 *      Author: m1ch1
 */

#include "PathFind_base.h"

namespace apps
{

PathFind_base::PathFind_base(GridMap* map)
{
   _map = map;
//   _robot_radius = 0;
}

PathFind_base::~PathFind_base()
{
}

void PathFind_base::setGridMap(GridMap* map)
{
   _map = map;
}

GridMap* PathFind_base::getGridMap() const
{
   return _map;
}

//void PathFind_base::binarize(uint8_t min, uint8_t max, uint8_t vel_in, uint8_t vel_out)
//{
//   unsigned int num = _map->getWidth() * _map->getHeight();
//   uint8_t* dat = _map->getMapData();
//   for(unsigned int i = 0; i < num; ++i)
//   {
//      if(dat[i] >= min && dat[i] <= max) //in range
//      {
//         dat[i] = vel_in;
//      }
//      else           //out range
//      {
//         dat[i] = vel_out;
//      }
//   }
//}

//void PathFind_base::inflate(uint8_t val, double offset)
//{
//   unsigned int num = _map->getWidth() * _map->getHeight();
//   unsigned int width = _map->getWidth();
//   unsigned int height = _map->getHeight();
//
//   uint8_t offset_ = offset / (double)_map->getCellSize() + 0.555;
//
//   uint8_t* tmp_map = new uint8_t[num];
//   uint8_t* target_map = _map->getMapData();
//   std::copy(_map->getMapData(), _map->getMapData() + num*sizeof(uint8_t),tmp_map);
//   //memcpy(tmp_map, _map->getMapData(),num * sizeof(uint8_t));
//
//   //read tmp_map ... write origin map
//   for(int i = 0; i < num; ++i)
//  {
//      //prove if sarounded by val ... after testing .. verry efficient
//      if(i > width && tmp_map[i-1] == val && tmp_map[i+1] == val && tmp_map[i - width] == val && tmp_map[i+width] == val)
//         continue;
//
//      if(tmp_map[i] == val) // get val points
//      {
//         int x = i % width;
//         int y = i / width;
//         for(int yi = y-offset_; yi < y+offset_; ++yi)
//         {
//            for(int xi = x - offset_; xi < x+offset_; ++xi)
//            {
//               int idx = yi * width + xi;
//               if(idx >= 0 && idx < num)
//                  target_map[idx] = val;
//            }
//         }
//      }
//   }
//   delete[] tmp_map;
//}


double PathFind_base::getPathLenght(std::vector<Point2D> path)
{
   if(!path.size())
      return 0.0;

   double length = 0;
   Point2D old;
   old = path[0];
   for(unsigned int i = 1; i < path.size(); ++i)
   {
      Point2D tmp;
      tmp.x = path[i].x - old.x;
      tmp.y = path[i].y - old.y;

      length += ::sqrt(tmp.x * tmp.x + tmp.y * tmp.y);

      old = path[i];
   }
   return length;
}


std::vector<Point2D> PathFind_base::computePathPoint(Point2D start, Point2D end)
{
   std::vector<Pixel> pixelPath = this->computePath(_map->toPixel(start), _map->toPixel(end));
   std::vector<Point2D> path(pixelPath.size());

   //convert to point2D path
   for(unsigned int i = 0; i < pixelPath.size(); ++i)
   {
      path[i] = _map->toPoint2D(pixelPath[i]);
   }
   return path;
}

bool PathFind_base::addCostmap(std::string key, GridMap* costmap)
{
   if(_map->isCompatible(costmap))
   {
      try{
         //if exist update
         _costMapStack.at(key) = costmap;
      }
      catch (const std::out_of_range& e) {
         //if not exist add one
         _costMapStack.insert(std::make_pair(key,costmap));
      }
      return true;
   }
   else
   {
      std::cout << "given costmap does not fit to intern map... not added" << std::endl;
      return false;
   }
}

GridMap* apps::PathFind_base::getCostmap(std::string key)
{
   try{
      return _costMapStack.at(key);
   }
   catch (const std::out_of_range& e) {
      return NULL;
   }
}

bool PathFind_base::removeCostmap(std::string key)
{
   try {
      _costMapStack.at(key);
      _costMapStack.erase(key);
      return true;
   } catch (const std::out_of_range& e) {
      return false;
   }
}

void PathFind_base::resetCostmaps()
{
   _costMapStack.clear();
}

} /* namespace apps */


