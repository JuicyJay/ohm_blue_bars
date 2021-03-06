/************************************************************************************************************
 * Class Map: For a easier handling of the ROS occupancy grid. A good feature is the constructor with a roi
 * functionality. This class is just a wrapper without dealing real data.
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___MAP_H___
#define ___MAP_H___

#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Core>

#include "Rect.h"

class Map
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Map(const unsigned int w = 0, const unsigned int h = 0, const float res = 0.01f);
    Map(nav_msgs::OccupancyGrid& map);
    Map(Map& map, const Rect& roi = Rect());
    virtual ~Map(void);

    inline unsigned int width(void) const { return _width; }
    inline unsigned int height(void) const { return _height; }
    inline float resolution(void) const { return _resolution; }
    inline const Eigen::Vector2f& origin(void) const { return _origin; }
    inline bool hasRoi(void) const { return _roi.isNull(); }
    inline const Rect& roi(void) const { return _roi; }
    inline bool isNull(void) const { return !_width || !_height; }

    inline int8_t& operator()(const unsigned int x, const unsigned int y)
    {
        return (*_data)[y * _stride + x + _offset];
    }
    inline const int8_t& operator()(const unsigned int x, const unsigned int y) const
    {
        return (*_data)[y * _stride + x + _offset];
    }

protected:
    bool                 _shared;
    unsigned int         _stride;
    unsigned int         _offset;

    unsigned int    _width;
    unsigned int    _height;
    float           _resolution;
    Eigen::Vector2f _origin;
    Rect            _roi;

private:
    std::vector<int8_t>* _data;
};

class ConstMap : public Map
{
public:
    ConstMap(const nav_msgs::OccupancyGrid& map, const Rect& roi = Rect());

    inline const int8_t& operator()(const unsigned int x, const unsigned int y) const
    {
        return (*_data)[y * _stride + x + _offset];
    }

private:
    const std::vector<int8_t>* _data;
};

#endif
