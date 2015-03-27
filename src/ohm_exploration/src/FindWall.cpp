#include "FindWall.h"

#include <ostream>

#include <ros/console.h>

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

void FindWall::setMap(const nav_msgs::OccupancyGrid& occu)
{
    nav_msgs::OccupancyGrid hack;
    hack = occu;

    if (_featureMap.isNull())
    {
        const Map map(hack);
        _featureMap.setMap(map);
    }
    else
    {
        const Rect roi(hack.info.width / 2, hack.info.height / 2, 200, 200);
	Map hack2(hack);
        const Map map(hack2, roi);
        _featureMap.updateMap(map);
    }

    for (unsigned int i = 0; i < _orientations.size(); ++i)
        _featureMap.exportPoints(_points[i], _orientations[i]);

    _mapMetaData = occu.info;
}

void FindWall::search(std::vector<Wall>& walls)
{
    /* Go through all set orientations and search for new walls. */
    for (unsigned int i = 0; i < _orientations.size(); ++i)
    {
        Wall wall;
        _ransac.setInputPoints(_points[i]);

        while (_ransac.estimateWall(wall))
        {
            if (wall.valid())
            {
                wall.setResolution(_mapMetaData.resolution);
                wall.setOrigin(_mapMetaData.origin.position);
                wall.setOrientation(_orientations[i]);
                walls.push_back(wall);

                ROS_INFO_STREAM("Found " << wall);
            }

            this->removePoints(wall.points(), _points[i]);
            _ransac.setInputPoints(_points[i]);
        }
    }

    /* Mark all found valid walls. This enures the walls can not be found again in the future. */
    _featureMap.markWalls(walls);
}

void FindWall::removePoints(const PointVector& remove, PointVector& points)
{
    std::vector<bool> mask(points.size(), true);
    PointVector rest;

    for (unsigned int i = 0; i < remove.size(); ++i)
        for (unsigned int j = 0; j < points.size(); ++j)
            mask[j] = mask[j] & (points[j] != remove[i]);

    for (unsigned int i = 0; i < mask.size(); ++i)
        if (mask[i])
            rest.push_back(points[i]);

    points = rest;
}
