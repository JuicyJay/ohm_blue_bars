#include "Map.h"


Map::Map(const unsigned int w, const unsigned int h, const float resolution)
    : _shared(false),
      _stride(w),
      _offset(0),
      _width(w),
      _height(h),
      _resolution(resolution),
      _origin(0.0f, 0.0f),
      _data(new std::vector<int8_t>(w * h))
{

}

Map::Map(nav_msgs::OccupancyGrid& map)
    : _shared(true),
      _stride(map.info.width),
      _offset(0),
      _width(map.info.width),
      _height(map.info.height),
      _resolution(map.info.resolution),
      _origin(map.info.origin.position.x, map.info.origin.position.y),
      _data(&map.data)
{

}

Map::Map(Map& map, const Rect& roi)
    : _shared(true),
      _roi(roi)
{
    if (!_roi.isNull())
    {
        _stride     = map._width;
        _offset     = map._width * _roi.y() + _roi.x();
        _data       = map._data;
        _width      = _roi.width();
        _height     = _roi.height();
        _resolution = map._resolution;
        _origin     = map._origin + Eigen::Vector2f(_roi.x(), _roi.y()) * _resolution;
    }
    else
    {
        _stride     = map._stride;
        _offset     = map._offset;
        _data       = map._data;
        _width      = map._width;
        _height     = map._height;
        _resolution = map._resolution;
        _origin     = map._origin;
    }
}

Map::~Map(void)
{
    if (!_shared)
        delete _data;
}



ConstMap::ConstMap(const nav_msgs::OccupancyGrid& map, const Rect& roi)
{
    _shared = true;
    _roi    = roi;

    if (!_roi.isNull())
    {
        _stride = map.info.width;
        _offset = map.info.width * _roi.y() + _roi.x();
        _width  = _roi.width();
        _height = _roi.height();
        _resolution = map.info.resolution;
        _origin     = Eigen::Vector2f(map.info.origin.position.x, map.info.origin.position.y) +
            Eigen::Vector2f(_roi.x(), _roi.y()) * _resolution;
        _data = &map.data;
    }
    else
    {
        _stride = map.info.width;
        _offset = 0;
        _width  = map.info.width;
        _height = map.info.height;
        _resolution = map.info.resolution;
        _origin.x() = map.info.origin.position.x;
        _origin.y() = map.info.origin.position.y;
        _data = &map.data;
    }
}
