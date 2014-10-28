#include "FindWall.h"

#include <ostream>

FindWall::FindWall(void)
    : _points(4)
{
    _ransac.setEpsilon(2.0f);
    _ransac.setMinimumPoints(40);
    _ransac.setMaxIterations(100);

    _orientations.push_back(Wall::Up);
    _orientations.push_back(Wall::Down);
    _orientations.push_back(Wall::Left);
    _orientations.push_back(Wall::Right);
}

void FindWall::setMap(const nav_msgs::OccupancyGrid& map)
{
    if (_featureMap.isNull())
        _featureMap.setMap(map);
    else
        _featureMap.updateMap(map);

    for (unsigned int i = 0; i < _orientations.size(); ++i)
        _featureMap.exportPoints(_points[i], _orientations[i]);

    _mapMetaData = map.info;
}

void FindWall::search(std::vector<Wall>& walls)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    for (unsigned int i = 0; i < _orientations.size(); ++i)
    {
        Wall wall;
        _ransac.setInputPoints(_points[i]);

        while (_ransac.estimateWall(wall))
        {
//            std::cout << "will work with " << _points[i].size() << " points." << std::endl;
//            std::cout << wall << std::endl;

            if (wall.valid())
            {
                wall.setResolution(_mapMetaData.resolution);
                wall.setOrigin(_mapMetaData.origin.position);
                wall.setOrientation(_orientations[i]);
                walls.push_back(wall);
            }

            this->removePoints(wall.points(), _points[i]);
            _ransac.setInputPoints(_points[i]);
        }
    }

    std::cout << "before mark walls." << std::endl;
    _featureMap.markWalls(walls);
    std::cout << "will exit from method search." << std::endl;
}

void FindWall::removePoints(const PointVector& remove, PointVector& points)
{
    std::vector<bool> mask(points.size(), true);
    PointVector rest;

    for (unsigned int i = 0; i < remove.size(); ++i)
    {
        for (unsigned int j = 0; j < points.size(); ++j)
        {
            mask[j] = mask[j] & (points[j] != remove[i]);
        }
    }

    for (unsigned int i = 0; i < mask.size(); ++i)
        if (mask[i])
            rest.push_back(points[i]);

    points = rest;
}
