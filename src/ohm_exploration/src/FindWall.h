/************************************************************************************************************
 * Class FindWall.
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___FIND_WALL_H___
#define ___FIND_WALL_H___

#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>

#include "Wall.h"
#include "Ransac.h"
#include "FeatureMap.h"

class FindWall
{
public:
    FindWall(void);

    void setMap(const nav_msgs::OccupancyGrid& map);
    void search(std::vector<Wall>& walls);
    cv::Mat getImageFromMap(void) const;

private:
    void removePoints(const PointVector& remove, PointVector& points);

    std::vector<PointVector>       _points;
    std::vector<Wall::Orientation> _orientations;
    nav_msgs::MapMetaData          _mapMetaData;

    Ransac     _ransac;
    FeatureMap _featureMap;
};

#endif
