#include "FeatureMap.h"

#include <iostream>

FeatureMap::FeatureMap(void)
{

}

void FeatureMap::setMap(const nav_msgs::OccupancyGrid& map)
{
    _data.resize(map.info.height);

    /* Check the changeovers in the map in x direction. */
    for (unsigned int row = 0; row < map.info.height; ++row)
    {
        const unsigned int offset = map.info.width * row;
        _data[row].resize(map.info.width);

        for (unsigned int col = 1; col < map.info.width; ++col)
        {
            const int8_t last = map.data[offset + col - 1];
            const int8_t current = map.data[offset + col];

            if (!last && current > 0)
                _data[row][col].orientation |= Wall::Left;
            else if (last > 0 && !current)
                _data[row][col - 1].orientation |= Wall::Right;
        }
    }


    /* Check the changeovers in the map in y direction. */
    for (unsigned int col = 0; col < map.info.width; ++col)
    {
        for (unsigned int row = 1; row < map.info.height; ++row)
        {
            const int8_t last = map.data[map.info.width * (row - 1) + col];
            const int8_t current = map.data[map.info.width * row + col];

            if (!last && current > 0)
                _data[row][col].orientation |= Wall::Up;
            else if (last > 0 && !current)
                _data[row - 1][col].orientation |= Wall::Down;
        }
    }
}
