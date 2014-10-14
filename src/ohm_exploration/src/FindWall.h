#ifndef ___FIND_WALL_H___
#define ___FIND_WALL_H___

#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include <Eigen/Core>

#include "Wall.h"
#include "Ransac.h"

class FindWall
{
public:
    FindWall(void);

    void setMap(const nav_msgs::OccupancyGrid& map);
    void search(std::vector<Wall>& walls);

private:
    void exportPoints(const nav_msgs::OccupancyGrid& map);
    void buildCluster(const nav_msgs::OccupancyGrid& map);

    PointVector _points;
    nav_msgs::MapMetaData _mapMetaData;
    Ransac _ransac;
};

#endif
