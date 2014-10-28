#ifndef ___MAP_H___
#define ___MAP_H___

#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Core>

#include "Rect.h"

class Map
{
public:
    Map(const unsigned int w = 0, const unsigned int h = 0, const float res = 0.01f);
    Map(nav_msgs::OccupancyGrid& map);
    Map(Map& map, const Rect& roi = Rect());
    virtual ~Map(void);

    inline unsigned int width(void) const { return _width; }
    inline unsigned int height(void) const { return _height; }
    inline float resolution(void) const { return _resolution; }
    inline const Eigen::Vector2f& origin(void) const { return _origin; }

    inline void setResolution(const float res) { _resolution = res; }
    inline void setOrigin(const Eigen::Vector2f& p) { _origin = p; }

    inline int8_t& operator()(const unsigned int x, const unsigned int y)
    {
        return (*_data)[y * _stride + x + _offset];
    }
    inline const int8_t& operator()(const unsigned int x, const unsigned int y) const
    {
        return (*_data)[y * _stride + x + _offset];
    }

private:
    bool _shared;
    unsigned int _stride;
    unsigned int _offset;
    std::vector<int8_t>* _data;

    unsigned int _width;
    unsigned int _height;
    float _resolution;
    Eigen::Vector2f _origin;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
