#ifndef ___FEATURE_MAP_H___
#define ___FEATURE_MAP_H___

#include <vector>

#include <nav_msgs/OccupancyGrid.h>

#include "FeatureCell.h"

class FeatureMap
{
public:
    FeatureMap(void);

    void setMap(const nav_msgs::OccupancyGrid& map);
    inline const FeatureCell& operator()(const unsigned int x, const unsigned int y) const { return _data[y][x]; }

private:
    std::vector<std::vector<FeatureCell> > _data;
};

#endif
