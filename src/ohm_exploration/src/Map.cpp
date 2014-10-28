#include "Map.h"

Map::Map(const unsigned int w, const unsigned int h, const float resolution)
    : _shared(false),
      _stride(w),
      _offset(0),
      _data(new std::vector<int8_t>(w * h)),
      _width(w),
      _height(h),
      _resolution(resolution),
      _origin(0.0f, 0.0f)
{

}

Map::Map(nav_msgs::OccupancyGrid& map)
    : _shared(true),
      _stride(map.info.width),
      _offset(0),
      _data(&map.data),
      _width(map.info.width),
      _height(map.info.height),
      _resolution(map.info.resolution),
      _origin(map.info.origin.position.x, map.info.origin.position.y)
{

}

Map::Map(Map& map, const Rect& roi)
    : _shared(true)
{
    if (roi.isNull())
    {
        _stride = map._stride;
        _offset = map._offset;
        _data = map._data;
        _width = map._width;
        _height = map._height;
        _resolution = map._resolution;
        _origin = map._origin;
    }
    else
    {
        _stride = map._width;
        _offset = map._width * roi.y() + roi.x();
        _data = map._data;
        _width = roi.width();
        _height = roi.height();
        _resolution = map._resolution;
        _origin = map._origin + Eigen::Vector2f(roi.x(), roi.y()) * _resolution;
    }
}

Map::~Map(void)
{
    if (!_shared)
        delete _data;
}
