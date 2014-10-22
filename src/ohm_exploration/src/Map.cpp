#include "Map.h"

Map::Map(const unsigned int w, const unsigned int h, const float resolution)
    : _shared(false),
      _data(new std::vector<int8_t>(w * h)),
      _resolution(resolution),
      _origin(0.0f, 0.0f)
{

}

Map::Map(nav_msgs::OccupancyGrid& map)
    : _shared(true),
      _data(&map.data),
      _resolution(map.info.resolution),
      _origin(map.info.origin.position.x, map.info.origin.position.y)
{

}

Map::Map(Map& map, const Rect& roi)
    : _shared(true)
{
    if (roi.isNull())
    {
        _data = map._data;
        _width = map._width;
        _height = map._height;
        _resolution = map._resolution;
        _origin = map._origin;
    }
    else
    {
        // to do
    }
}

Map::~Map(void)
{
    if (!_shared)
        delete _data;
}
