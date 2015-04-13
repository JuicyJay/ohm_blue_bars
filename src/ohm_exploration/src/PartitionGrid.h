/************************************************************************************************************
 * Class PartitionGrid:
 *
 *  Created on: 13.04.2015
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___PARTITION_GRID_H___
#define ___PARTITION_GRID_H___

#include <vector>

#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>

#include "Partition.h"

class PartitionGrid
{
public:
    PartitionGrid(const nav_msgs::OccupancyGrid& map, const float cellsize = 1.2f);

    visualization_msgs::MarkerArray getMarkerMsg(void) const;

private:
    std::vector<Partition> _grid;
};

#endif
