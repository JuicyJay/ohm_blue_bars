#ifndef ___FIND_WALL_H___
#define ___FIND_WALL_H___

#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include <Eigen/Core>

#include "Wall.h"
#include "Ransac.h"
#include "FeatureMap.h"

class FindWall
{
public:
    FindWall(void);

    void setMap(const nav_msgs::OccupancyGrid& map);
    void search(std::vector<Wall>& walls);

private:
    void exportPoints(const nav_msgs::OccupancyGrid& map,
                      PointVector& points,
                      const Wall::Orientation orientation = Wall::All);
    void removePoints(const PointVector& remove, PointVector& points);
    void buildCluster(const nav_msgs::OccupancyGrid& map);

    std::vector<PointVector> _points;
    std::vector<Wall::Orientation> _orientations;
    nav_msgs::MapMetaData _mapMetaData;
    Ransac _ransac;
    FeatureMap _featureMap;
};

#endif
