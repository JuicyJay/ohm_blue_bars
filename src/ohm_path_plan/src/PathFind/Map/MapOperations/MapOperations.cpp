/*
 * MapOperations.cpp
 *
 *  Created on: 29.12.2014
 *      Author: m1ch1
 */

#include "MapOperations.h"
#include <iostream>

namespace apps
{

void MapOperations::binarize(GridMap* map,
                             uint8_t min,
                             uint8_t max,
                             uint8_t vel_in,
                             uint8_t vel_out)
{
   unsigned int num = map->getWidth() * map->getHeight();
   uint8_t* dat = map->getMapData();
   for(unsigned int i = 0; i < num; ++i)
   {
      if(dat[i] >= min && dat[i] <= max) //in range
      {
         dat[i] = vel_in;
      }
      else           //out range
      {
         dat[i] = vel_out;
      }
   }
}

void MapOperations::inflate(GridMap* map, uint8_t val_min, uint8_t val_max, double offset)
{
   unsigned int num = map->getWidth() * map->getHeight();
   unsigned int width = map->getWidth();
   unsigned int height = map->getHeight();

   uint8_t offset_ = offset / (double)map->getCellSize() + 0.555;

   uint8_t* tmp_map = new uint8_t[num];
   uint8_t* target_map = map->getMapData();
   std::copy(map->getMapData(), map->getMapData() + num*sizeof(uint8_t),tmp_map);
   //memcpy(tmp_map, _map->getMapData(),num * sizeof(uint8_t));

   //read tmp_map ... write origin map
   for(int i = 0; i < num; ++i)
   {
      //prove if sarounded by val ... after testing .. verry efficient
      if(i > width && i < (width * (height - 1)) &&
         tmp_map[i-1] >= val_min && tmp_map[i-1] <= val_max &&
         tmp_map[i+1] >= val_min && tmp_map[i+1] <= val_max &&
         tmp_map[i - width] >= val_min && tmp_map[i - width] <= val_max &&
         tmp_map[i+width] >= val_min && tmp_map[i+width] <= val_max)
         continue;

      if(tmp_map[i] >= val_min && tmp_map[i] <= val_max) // get val points
      {
         int x = i % width;
         int y = i / width;
         for(int yi = y-offset_; yi < y+offset_; ++yi)
         {
            for(int xi = x - offset_; xi < x+offset_; ++xi)
            {
               int idx = yi * width + xi;
               if(idx >= 0 && idx < num)
                  target_map[idx] = tmp_map[i];
            }
         }
      }
   }
   delete[] tmp_map;
}

void MapOperations::distnaceTransform(GridMap* map, double offset, uint8_t wall_val)
{
   unsigned int num = map->getWidth() * map->getHeight();
   unsigned int width = map->getWidth();
   unsigned int height = map->getHeight();

   uint8_t offset_ = offset / (double)map->getCellSize() + 0.555;

   uint8_t* dat = map->getMapData();

   //compute loockuptable:
   int x = offset_;   //middle point
   int y = offset_;   //middle point
   unsigned char* lt = new unsigned char[(2 * offset_ + 1) * (2 * offset_ + 1)];

   for(unsigned int i = 0; i < (2 * offset_ + 1) * (2 * offset_ + 1); ++i)
   {
      lt[i] = 0;
   }

   for(unsigned int off = 0; off <= offset_; ++off)
   {
      int val = (offset_ - off) * (255 / offset_);
      val = (val > 254) ? 254 : val;
      for(unsigned int yi = y - off; yi < y + off ; ++yi)
      {
         for(unsigned int xi = x - off; xi < x + off; ++xi)
         {
            int idx = yi * (2* offset_ +1) + xi;
            lt[idx] = (lt[idx] > val) ? lt[idx] : val;
         }
      }
   }

   //do distance transform:
   for(int i = 0; i < num; ++i)
   {
      if(dat[i] == wall_val) // get wall points
      {
         int x = i % width;
         int y = i / width;

         //prove if sarounded by wall ... after testing .. verry efficient
         if(i > width && i < (width * (height - 1)) && dat[i-1] == wall_val && dat[i+1] == wall_val && dat[i - width] == wall_val && dat[i+width] == wall_val)
            continue;

         for(int yi = y-offset_, ylt = 0; yi < y+offset_; ++yi, ++ylt)
         {
            int idx_ = yi * width;
            int idxlt_ = ylt * (2 * offset_ + 1);
            for(int xi = x - offset_, xlt = 0; xi < x+offset_; ++xi, ++xlt)
            {
               int idx = idx_ + xi;
               int idxlt = idxlt_ + xlt;

               if(idx >= 0 && idx < num)
               {
                  if(dat[idx] < lt[idxlt])
                  {
                     dat[idx] = lt[idxlt];
                  }
               }
            }
         }
      }
   }

   delete[] lt;
}

} /* namespace apps */


