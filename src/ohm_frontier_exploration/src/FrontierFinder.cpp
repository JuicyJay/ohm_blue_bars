/*
 * FrontierFinder.cpp
 *
 *  Created on: 26.01.2015
 *      Author: chris
 */

#include "FrontierFinder.h"

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <tf/tf.h>
#include <tf/LinearMath/Vector3.h>

#include <math.h>       /* atan2 */

namespace autonohm {

FrontierFinder::FrontierFinder(void) :
      _initialized(false)
{
   // start with no initialization
}

FrontierFinder::FrontierFinder(FrontierFinderConfig config) :
      _initialized(true)
    , _config(config)
{

}

FrontierFinder::~FrontierFinder(void)
{
   // nothing to do
}

void FrontierFinder::setMap(const nav_msgs::OccupancyGrid& map)
{
   _map = map;
}

void FrontierFinder::setConfig(FrontierFinderConfig config)
{
   _config = config;
}

std::vector<Frontier> FrontierFinder::getFrontiers(void)
{
   return _frontiers;
}

void FrontierFinder::calculateFrontiers(void)
{
   // remove all old frontiers
   _frontiers.clear();

   int idx;
   const int w    = _map.info.width;
   const int size = _map.info.height * w;


   std::cout << __PRETTY_FUNCTION__ <<  " 1" << std::endl;

   // copy to tmp array
   signed char* map = new signed char[size];
   for(unsigned int idx=0 ; idx<size ; idx++)
      map[idx] = _map.data[idx];

   /*
    * Find all frontiers
    */
   for (idx = 0; idx<size; idx++)
   {
      const bool valid_point = map[idx] < 100 ;

      if ((valid_point  && map)
                        && (((idx + 1 < size) && (map[idx + 1] == -1))
                        ||  ((idx - 1 >= 0)   && (map[idx - 1] == -1))
                        ||  ((idx + w < size) && (map[idx + w] == -1))
                        ||  ((idx - w >= 0)   && (map[idx - w] == -1))))
      {
         _map.data[idx] = -128;
      }
      else
      {
         _map.data[idx] = -127;
      }
   }


   // clean up frontiers on seperate rows of the map
   idx = _map.info.height - 1;
   for (unsigned int y = 0; y < _map.info.width; y++) {
      _map.data[idx] = -127;
      idx += _map.info.height;
   }

   std::cout << "Size: " << size << std::endl;

   // group frontiers
   int segment_id = 127;
   std::vector<std::vector<FrontierPoint> > segments;

   for (unsigned int i = 0; i < size; i++)
   {
      if (_map.data[i] == -128)
      {
         std::vector<int> neighbors;
         std::vector<FrontierPoint> segment;
         neighbors.push_back(i);

         std::cout <<"neightbors size: " << neighbors.size() << std::endl;

         while (neighbors.size() > 0)
         {
            int idx = neighbors.back();
            neighbors.pop_back();
            _map.data[idx] = segment_id;

            tf::Vector3 orientation(0, 0, 0);
            int c = 0;
            if ((idx + 1 < size) && (map[idx+1] == -1)) {
               orientation += tf::Vector3(1, 0, 0); c++;
            }
            if ((idx-1 >= 0)   && (map[idx-1]   == -1)) {
               orientation += tf::Vector3(-1, 0, 0); c++;
            }
            if ((idx+w < size) && (map[idx+w]   == -1)) {
               orientation += tf::Vector3(0, 1, 0);  c++;
            }
            if ((idx-w >= 0)   && (map[idx-w]   == -1)) {
               orientation += tf::Vector3(0, -1, 0); c++;
            }


            assert(c > 0);


            tf::Vector3 frontierOrientation;
            autonohm::FrontierPoint fp;
            fp.idx         = idx;
            fp.orientation = frontierOrientation / 3;
            segment.push_back(fp);


            // check all 8 neighbors
            if (((idx - 1) > 0)                  && (_map.data[idx - 1]     == -128))
               neighbors.push_back(idx - 1);

            if (((idx + 1) < size)               && (_map.data[idx + 1]     == -128))
               neighbors.push_back(idx + 1);

            if (((idx - w) > 0)                  && (_map.data[idx - w]     == -128))
               neighbors.push_back(idx - w);

            if (((idx - w + 1) > 0)              && (_map.data[idx - w + 1] == -128))
               neighbors.push_back(idx - w + 1);

            if (((idx - w - 1) > 0)              && (_map.data[idx - w - 1] == -128))
               neighbors.push_back(idx - w - 1);

            if (((idx + w) < size)               && (_map.data[idx + w]     == -128))
               neighbors.push_back(idx + w);

            if (((idx + w + 1) < size)           && (_map.data[idx + w + 1] == -128))
               neighbors.push_back(idx + w + 1);

            if (((idx + w - 1) < size)           && (_map.data[idx + w - 1] == -128))
               neighbors.push_back(idx + w - 1);
         }

         segments.push_back(segment);
         segment_id--;
         if (segment_id < -127)
            break;
      }
//      else
//      {
//         std::cout << __PRETTY_FUNCTION__ << " error" << std::endl;
//      }
   }

   int num_segments = 127 - segment_id;
   if (num_segments <= 0)
      return;

   ROS_DEBUG_STREAM("Found " << segments.size() << " frontieres. ");

   for (unsigned int i = 0; i < segments.size(); i++)
   {

      std::vector<FrontierPoint>& segment = segments[i];
      uint size = segment.size();

      std::cout << "segment size: " << segment.size() << std::endl;

      // check for segment size
//      if (size < _config.robot_radius)
//         continue;

      float x = 0;
      float y = 0;
      tf::Vector3 d(0, 0, 0);
      for (unsigned int j = 0; j < size; j++)
      {
         d += segment[j].orientation;

         int idx = segment[j].idx;
         x += (idx % _map.info.width);
         y += (idx / _map.info.width);

      }

      std::cout << "x: " << x << " y: " << y << std::endl;

      d = d / size;

      Frontier f;
      f.position.x = _map.info.origin.position.x + _map.info.resolution * (x / size);
      f.position.y = _map.info.origin.position.y + _map.info.resolution * (y / size);
      f.position.z = 0.0;

      f.orientation = tf::createQuaternionMsgFromYaw(std::atan2(d.y(), d.x()));

      _frontiers.push_back(f);
   }
}

} /* namespace autonohm */
