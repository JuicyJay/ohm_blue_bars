#include "FindWall.h"

#include <ostream>

FindWall::FindWall(void)
{
    _ransac.setEpsilon(2.0f);
    _ransac.setMinimumPoints(30);
    _ransac.setMaxIterations(100);
}

void FindWall::setMap(const nav_msgs::OccupancyGrid& map)
{
    _points.clear();
    this->exportPoints(map);
    _mapMetaData = map.info;
}

void FindWall::search(std::vector<Wall>& walls)
{
    Wall wall;

    _ransac.setInputPoints(_points);
    _ransac.estimateWall(wall);

    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::cout << wall << std::endl;
}

void FindWall::exportPoints(const nav_msgs::OccupancyGrid& map)
{
    for (unsigned int row = 0; row < map.info.height; ++row)
    {
        const unsigned int offset = map.info.width * row;

        for (unsigned int col = 0; col < map.info.width; ++col)
            if (map.data[offset + col] > 0)
                _points.push_back(Eigen::Vector2i(col, row));
    }
}
