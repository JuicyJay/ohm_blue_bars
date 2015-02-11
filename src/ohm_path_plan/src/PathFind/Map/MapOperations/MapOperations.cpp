/*
 * MapOperations.cpp
 *
 *  Created on: 29.12.2014
 *      Author: m1ch1
 */

#include "MapOperations.h"
#include <iostream>
//#include <opencv2/opencv.hpp>

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

void MapOperations::inflateRect(GridMap* map, uint8_t val_min, uint8_t val_max, double offset)
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

void MapOperations::distnaceTransformRect(GridMap* map, double offset, uint8_t wall_val)
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


void apps::MapOperations::inflateCirc(GridMap* map, uint8_t val_min, uint8_t val_max, double offset)
{
   unsigned int num = map->getWidth() * map->getHeight();
   unsigned int width = map->getWidth();
   unsigned int height = map->getHeight();

   uint8_t offset_ = offset / (double)map->getCellSize() + 0.555;


   uint8_t* tmp_map = new uint8_t[num];
   uint8_t* target_map = map->getMapData();
   std::copy(map->getMapData(), map->getMapData() + num*sizeof(uint8_t),tmp_map);


   //compute loockuptable:
   int x = offset_;   //middle point
   int y = offset_;   //middle point
   unsigned int width_lt = 2 * offset_ +1;
   uint8_t* lt = new uint8_t[width_lt * width_lt];

   for(unsigned int i = 0; i < (2 * offset_ + 1) * (2 * offset_ + 1); ++i)
   {
      lt[i] = 0;
   }

   Pixel c(x,y);

   //draw filled circ in lt
   for(unsigned int i = 0; i < width_lt * width_lt; ++i)
   {
      Pixel p = MapOperations::idxToPixel(i,width_lt);
      //prove if point is in circ
      if((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y) < ((offset_+1)*(offset_+1)))
      {
         lt[i] = val_max;
      }
   }


   //do inflation
   for(int i = 0; i < num; ++i)
   {
      if(tmp_map[i] >= val_min && tmp_map[i] <= val_max) // get wall points
      {
         int x = i % width;
         int y = i / width;

         //prove if sarounded by val ... after testing .. verry efficient
         if(i > width && i < (width * (height - 1)) &&
            tmp_map[i-1] >= val_min && tmp_map[i-1] <= val_max &&
            tmp_map[i+1] >= val_min && tmp_map[i+1] <= val_max &&
            tmp_map[i - width] >= val_min && tmp_map[i - width] <= val_max &&
            tmp_map[i+width] >= val_min && tmp_map[i+width] <= val_max)
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
                  if(tmp_map[idx] < lt[idxlt])
                  {
                     target_map[idx] = lt[idxlt];
                  }
               }
            }
         }
      }
   }


//   cv::Mat mat_lt(cv::Size(width_lt, width_lt),
//                  CV_8UC1,
//                  (unsigned char*) lt,
//                  cv::Mat::AUTO_STEP);
//
//   cv::imwrite("/tmp/lt_circ.png", mat_lt);

   delete[] tmp_map;
   delete[] lt;
}

void apps::MapOperations::distnaceTransformCirc(GridMap* map, double offset, uint8_t wall_val)
{
   unsigned int num = map->getWidth() * map->getHeight();
   unsigned int width = map->getWidth();
   unsigned int height = map->getHeight();

   uint8_t offset_ = offset / (double)map->getCellSize() + 0.555;

   if(offset_ == 0)
   {//nothing to do
      return;
   }

   uint8_t* tmp_map = new uint8_t[num];
   uint8_t* target_map = map->getMapData();
   std::copy(map->getMapData(), map->getMapData() + num*sizeof(uint8_t),tmp_map);


   uint8_t* dat = map->getMapData();

   //compute loockuptable:
   int x = offset_;   //middle point
   int y = offset_;   //middle point
   unsigned int width_lt = 2 * offset_ +1;
   uint8_t* lt = new uint8_t[width_lt * width_lt];

   for(unsigned int i = 0; i < (2 * offset_ + 1) * (2 * offset_ + 1); ++i)
   {
      lt[i] = 0;
   }

   Pixel c(x,y);

   for(unsigned int i = 0; i < width_lt * width_lt; ++i)
   {
      Pixel p = MapOperations::idxToPixel(i,width_lt);
      //prove if point is in circ
      if((p.x - x)*(p.x - x) + (p.y - y)*(p.y - y) < ((offset_+1)*(offset_+1)))
      {
         //get euclidian distance and normalize to 1
         Point2D tmp;
         tmp.x = x - p.x;
         tmp.y = y - p.y;

         double d = ::sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
         d /= (double)(offset_ + 1) + 0.555;

         lt[i] = (uint8_t)(255 * (1 - d));
      }
   }


   //do dt
   for(int i = 0; i < num; ++i)
   {
      if(tmp_map[i] == wall_val) // get wall points
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
                  if(target_map[idx] < lt[idxlt])
                  {
                     target_map[idx] = lt[idxlt];
                  }
               }
            }
         }
      }
   }

//   cv::Mat mat_lt(cv::Size(width_lt, width_lt),
//                  CV_8UC1,
//                  (unsigned char*) lt,
//                  cv::Mat::AUTO_STEP);
//
//   cv::imwrite("/tmp/lt_dt_circ.png", mat_lt);

   delete[] tmp_map;
   delete[] lt;
}

void MapOperations::drawCircle(uint8_t* data, unsigned int width, Pixel c, unsigned int radius, uint8_t value)
{
   int x0 = c.x;
   int y0 = c.y;

   int f = 1 - radius;
   int ddF_x = 0;
   int ddF_y = -2 * radius;
   int x = 0;
   int y = radius;

//   setPixel(x0, y0 + radius);
//   setPixel(x0, y0 - radius);
//   setPixel(x0 + radius, y0);
//   setPixel(x0 - radius, y0);
   data[MapOperations::pixelToIdx(Pixel(x0, y0 + radius), width)] = value;
   data[MapOperations::pixelToIdx(Pixel(x0, y0 - radius), width)] = value;
   data[MapOperations::pixelToIdx(Pixel(x0 + radius, y0), width)] = value;
   data[MapOperations::pixelToIdx(Pixel(x0 - radius, y0), width)] = value;

   while(x < y)
   {
      if(f >= 0)
      {
         y--;
         ddF_y += 2;
         f += ddF_y;
      }
      x++;
      ddF_x += 2;
      f += ddF_x + 1;

//      setPixel(x0 + x, y0 + y);
//      setPixel(x0 - x, y0 + y);
//      setPixel(x0 + x, y0 - y);
//      setPixel(x0 - x, y0 - y);
//      setPixel(x0 + y, y0 + x);
//      setPixel(x0 - y, y0 + x);
//      setPixel(x0 + y, y0 - x);
//      setPixel(x0 - y, y0 - x);

      data[MapOperations::pixelToIdx(Pixel(x0 + x, y0 + y), width)] = value;
      data[MapOperations::pixelToIdx(Pixel(x0 - x, y0 + y), width)] = value;
      data[MapOperations::pixelToIdx(Pixel(x0 + x, y0 - y), width)] = value;
      data[MapOperations::pixelToIdx(Pixel(x0 - x, y0 - y), width)] = value;
      data[MapOperations::pixelToIdx(Pixel(x0 + y, y0 + x), width)] = value;
      data[MapOperations::pixelToIdx(Pixel(x0 - y, y0 + x), width)] = value;
      data[MapOperations::pixelToIdx(Pixel(x0 + y, y0 - x), width)] = value;
      data[MapOperations::pixelToIdx(Pixel(x0 - y, y0 - x), width)] = value;
   }
}

} /* namespace apps */


