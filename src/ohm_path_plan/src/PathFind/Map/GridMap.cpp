/*
 * GridMap.cpp
 *
 *  Created on: 14.12.2014
 *      Author: m1ch1
 */

#include "GridMap.h"

namespace apps
{

GridMap::GridMap(uint8_t* data, unsigned int width, unsigned int height, double cellSize, Point2D origin)
{
   //copy data into own mem
   _map = new uint8_t[width * height];
   std::copy(data, data + width*height * sizeof(uint8_t), _map);
   //memcpy(_map,data,width*height*sizeof(uint8_t));

   _width = width;
   _height = height;
   _cellSize = cellSize;
   _origin.x = origin.x / _cellSize + 0.555;
   _origin.y = origin.y / _cellSize + 0.555;
}

apps::GridMap::GridMap(GridMap* map)
{
   //copy data into own mem
   _map = new uint8_t[map->getWidth() * map->getHeight()];
   std::copy(map->getMapData(), map->getMapData() + map->getWidth() * map->getHeight() * sizeof(uint8_t), _map);
   //memcpy(_map,data,width*height*sizeof(uint8_t));

   _width = map->getWidth();
   _height = map->getHeight();
   _cellSize = map->getCellSize();
   _origin = map->getOriginPixel();
}

GridMap::~GridMap()
{
   delete[] _map;
}

uint8_t* GridMap::getMapData() const
{
   return _map;
}

int GridMap::updateData(uint8_t* data, unsigned int width, unsigned int height)
{
   if(width != _width || height != _height)
   {
      delete[] _map;
      _map = new uint8_t[width * height];
      _width = width;
      _height = height;
   }
   std::copy(data, data + width*height * sizeof(uint8_t), _map);
   //memcpy(_map,data,width*height*sizeof(uint8_t));

   return 0;
}

double GridMap::getCellSize() const
{
   return _cellSize;
}

unsigned int GridMap::getWidth() const
{
   return _width;
}

unsigned int GridMap::getHeight() const
{
   return _height;
}


void GridMap::setOrigin(Pixel origin)
{
   _origin = origin;
}

void GridMap::setOrigin(Point2D origin)
{
   //_origin = this->toPixel(origin);
   _origin.x = origin.x / _cellSize + 0.555;
   _origin.y = origin.y / _cellSize + 0.555;
}

Point2D GridMap::getOrigin()
{
   Point2D tmp;
   tmp.x = (double)_origin.x * _cellSize;
   tmp.y = (double)_origin.y * _cellSize;
   return tmp;
}

Pixel GridMap::getOriginPixel() const
{
   return _origin;
}

Pixel GridMap::toPixel(Point2D point)
{
   Pixel tmp;
   tmp.x = point.x / _cellSize + 0.555;
   tmp.y = point.y / _cellSize + 0.555;

   //translate to origin
   tmp.x -= _origin.x;
   tmp.y -= _origin.y;

   return tmp;
}



Point2D GridMap::toPoint2D(Pixel pixel)
{
   Point2D tmp;
   tmp.x = (double)pixel.x * _cellSize;
   tmp.y = (double)pixel.y * _cellSize;

   tmp.x += (double)_origin.x * _cellSize;
   tmp.y += (double)_origin.y * _cellSize;
   return tmp;
}

bool apps::GridMap::isCompatible(const GridMap* const comp_map) const
{
   if(this->isEqual(comp_map->getCellSize(),this->getCellSize()) &&
      comp_map->getWidth()    == this->getWidth()    &&
      comp_map->getHeight()   == this->getHeight() )
   {
      return true;
   }
   else
   {
      return false;
   }
}


} /* namespace apps */


