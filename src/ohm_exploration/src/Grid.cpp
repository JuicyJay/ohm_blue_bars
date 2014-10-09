#include "Grid.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

Grid::Grid(const unsigned int width, const unsigned int height, const float cellSize)
    : _data(height, std::vector<Cell>(width)),
      _cellSize(cellSize)
{

}

nav_msgs::GridCells Grid::getGridCellMessage(void) const
{
    const float offsetX = static_cast<float>(_data.size()) * 0.5f * _cellSize;
    const float offsetY = static_cast<float>(_data.size() > 0 ? _data[0].size() : 0.0f) * 0.5f * _cellSize;
    nav_msgs::GridCells message;

    message.header.frame_id = "map";
    message.header.stamp    = ros::Time::now();
    message.cell_width      = _cellSize;
    message.cell_height     = _cellSize;

    for (unsigned int row = 0; row < _data.size(); ++row)
    {
        for (unsigned int col = 0; col < _data[row].size(); ++col)
        {
            if (_data[row][col].inspected)
                continue;

            geometry_msgs::Point point;

            point.x = static_cast<float>(col) * _cellSize - offsetX;
            point.y = static_cast<float>(row) * _cellSize - offsetY;
            point.z = 0.0f;
            message.cells.push_back(point);
        }
    }

    return message;
}

void Grid::update(const nav_msgs::OccupancyGrid& map, const Sensor& sensor, const tf::Transform& pose)
{
    const Eigen::Vector2f originMap(map.info.origin.position.x, map.info.origin.position.y);
    const Eigen::Vector2f origin(Eigen::Vector2f(0.0f, 0.0f) - originMap);
    const Eigen::Vector2f cell(origin / map.info.resolution);
    const Eigen::Quaternionf orientation(pose.getRotation().w(),
                                         pose.getRotation().x(),
                                         pose.getRotation().y(),
                                         pose.getRotation().z());
    const Eigen::Vector3f direction(orientation * Eigen::Vector3f(1.0f, 0.0f, 0.0f));
    const float angle = std::acos(direction[0]);
    const float step = std::sin(angle) / map.info.resolution;

    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::cout << "orienation:" << std::endl << orientation.matrix() << std::endl;
    std::cout << "direction:" << std::endl << direction << std::endl;
    std::cout << "angle = " << angle << std::endl;
    std::cout << "sin(angle) = " << std::sin(angle) << std::endl;
    std::cout << "resolution = " << map.info.resolution << std::endl;
    std::cout << "step = " << step << std::endl;
}
