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

    inline void setResolution(const float res) { _resolution = res; }
    inline void setOrigin(const Eigen::Vector2f& p) { _origin = p; }
    inline float resolution(void) const { return _resolution; }
    inline const Eigen::Vector2f& origin(void) const { return _origin; }

    inline int8_t& operator()(const unsigned int x, const unsigned int y) { return (*_data)[y * _width + x]; }
    inline const int8_t& operator()(const unsigned int x, const unsigned int y) const
    {
        return (*_data)[y * _width + x];
    }

private:
    bool _shared;
    std::vector<int8_t>* _data;

    unsigned int _width;
    unsigned int _height;
    float _resolution;
    Eigen::Vector2f _origin;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
