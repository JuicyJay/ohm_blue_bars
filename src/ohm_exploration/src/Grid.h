#ifndef ___GRID_H___
#define ___GRID_H___

#include <nav_msgs/GridCells.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

#include <vector>

#include "Cell.h"
#include "Sensor.h"

class Grid
{
public:
    Grid(const nav_msgs::OccupancyGrid& map, const unsigned int fuseCells = 4);

    nav_msgs::GridCells getGridCellMessage(void) const;
    void update(const nav_msgs::OccupancyGrid& map, const Sensor& sensor, const tf::Transform& pose);

private:
    std::vector<std::vector<Cell> > _data;
    const float _cellSize;
    const unsigned int _fuseCells;
};

#endif
