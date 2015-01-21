/*
 * GridMap.h
 *
 *  Created on: 14.12.2014
 *      Author: m1ch1
 */

#ifndef ALGORITHM_PATHFIND_MAP_GRIDMAP_H_
#define ALGORITHM_PATHFIND_MAP_GRIDMAP_H_

#include <stdint.h>
#include <algorithm>
#include <cmath>
#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#endif

namespace apps
{

struct Pixel{
   unsigned int x;
   unsigned int y;

   Pixel() : x(0), y(0) { }
   Pixel(unsigned int _x, unsigned int _y) : x(_x) , y(_y) { }
};

typedef struct{
   double x;
   double y;
} Point2D;

/**
 * @brief
 *
 * @todo remove map operating stuff.... that should be in base pathplan class
 */
class GridMap
{
public:
   GridMap(uint8_t* data, unsigned int width, unsigned int height, double cellSize, Point2D origin);
   /**
    * copy
    * @param map
    */
   GridMap(GridMap* map);
   virtual ~GridMap();

   uint8_t* getMapData() const;
   /**
    * @brief updates data of map
    *
    * @param data    ptr to data
    * @param width   of data
    * @param height  of data
    * @return 0 on succes and -1 on error(maybe dimensions error)
    */
   int updateData(uint8_t* data, unsigned int width, unsigned int height);

   double getCellSize() const;
   unsigned int getWidth() const;
   unsigned int getHeight() const;

   void setOrigin(Pixel origin);
   void setOrigin(Point2D origin);
   Point2D getOrigin();
   Pixel getOriginPixel() const;

   /**
    * @todo compute with origin
    * @param point
    * @return
    */
   Pixel toPixel(Point2D point);
   /**
    * @todo compute with origin
    * @param pixel
    * @return
    */
   Point2D toPoint2D(Pixel pixel);

   /**
    * @brief proves if map properies (width, height, cellsize) is Equal
    *
    * @param comp_map -> map to compate
    * @return true if properties fit, flase if not
    */
   bool isCompatible(const GridMap* const comp_map) const;


#ifdef USE_OPENCV
cv::Mat toCvMat()
{
   cv::Mat tmp_img(cv::Size(this->getWidth(), this->getHeight()),
                   CV_8UC1,
                   (unsigned char*) this->getMapData(),
                   cv::Mat::AUTO_STEP);
   return tmp_img;
}
#endif


private: // functions
   inline bool isEqual(double x, double y) const
   {
     const double epsilon = 0.000001;
     return std::abs(x - y) <= epsilon * std::abs(x);
   }


private:
   uint8_t* _map;

   unsigned int _width;
   unsigned int _height;
   double _cellSize;

   Pixel _origin;
};

} /* namespace apps */

#endif /* ALGORITHM_PATHFIND_MAP_GRIDMAP_H_ */
