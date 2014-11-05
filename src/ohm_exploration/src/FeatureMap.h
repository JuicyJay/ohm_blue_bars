#ifndef ___FEATURE_MAP_H___
#define ___FEATURE_MAP_H___

#include <vector>

#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/opencv.hpp>

#include "FeatureCell.h"
#include "Wall.h"

class FeatureMap
{
public:
    FeatureMap(void);

    void setMap(const nav_msgs::OccupancyGrid& map);
    void updateMap(const nav_msgs::OccupancyGrid& map);

    void exportPoints(PointVector& points, const Wall::Orientation orientation = Wall::All);
    void markWalls(const std::vector<Wall>& walls);

    void paintImage(cv::Mat& image, const FeatureCell must);

    inline unsigned int width(void) const { return _width; }
    inline unsigned int height(void) const { return _height; }
    inline bool isNull(void) const { return !_data.size(); }
    inline const FeatureCell& operator()(const unsigned int x, const unsigned int y) const { return _data[y][x]; }

private:
    std::vector<std::vector<FeatureCell> > _data;
    unsigned int _wallDepth;
    unsigned int _width;
    unsigned int _height;
};

#endif
