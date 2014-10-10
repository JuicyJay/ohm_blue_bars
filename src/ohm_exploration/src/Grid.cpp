#include "Grid.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

Grid::Grid(const nav_msgs::OccupancyGrid& map, const unsigned int fuseCells)
    : _data(map.info.height / fuseCells, std::vector<Cell>(map.info.width / fuseCells)),
      _cellSize(map.info.resolution * fuseCells),
      _fuseCells(fuseCells)
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
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    const Eigen::Vector2f originMap(map.info.origin.position.x, map.info.origin.position.y);
    const Eigen::Vector2f position(pose.getOrigin()[0], pose.getOrigin()[1]);
    const Eigen::Vector2f origin(position - originMap);
    const Eigen::Vector2f cell(origin / map.info.resolution);
    const Eigen::Quaternionf orientation(pose.getRotation().w(),
                                         pose.getRotation().x(),
                                         pose.getRotation().y(),
                                         pose.getRotation().z());

    for (float angle = -sensor.beamAngleH() * 0.5f;
         angle <= sensor.beamAngleH() * 0.5f;
         angle += sensor.beamAngleH() / 11.0f)
    {
        const Eigen::Quaternionf beamOrientation(Eigen::AngleAxisf(angle, Eigen::Vector3f::UnitZ()));
        const Eigen::Vector3f direction(orientation * beamOrientation * Eigen::Vector3f(1.0f, 0.0f, 0.0f));
        const Eigen::Vector2f delta(std::sqrt(1 + (direction[1] * direction[1]) / (direction[0] * direction[0])),
                                    std::sqrt(1 + (direction[0] * direction[0]) / (direction[1] * direction[1])));
        const float range = sensor.range() / std::cos(std::abs(angle));
//    std::cout << "direction:" << std::endl << direction << std::endl;
//    std::cout << "delta:" << std::endl << delta << std::endl;
//        std::cout << "range = " << range << std::endl;

        Eigen::Vector2i step;
        Eigen::Vector2f sideDistance;

        if (direction[0] < 0)
        {
            step[0] = -1;
            sideDistance[0] = (cell[0] - static_cast<float>(static_cast<int>(cell[0]))) * delta[0];
        }
        else
        {
            step[0] = 1;
            sideDistance[0] = (static_cast<float>(static_cast<int>(cell[0]) + 1) - cell[0]) * delta[0];
        }
        if (direction[1] < 0)
        {
            step[1] = -1;
            sideDistance[1] = (cell[1] - static_cast<float>(static_cast<int>(cell[1]))) * delta[1];
        }
        else
        {
            step[1] = 1;
            sideDistance[1] = (static_cast<float>(static_cast<int>(cell[1]) + 1) - cell[1]) * delta[1];
        }

//    std::cout << "cell:" << std::endl << cell << std::endl;
//    std::cout << "step:" << std::endl << step << std::endl;
//    std::cout << "sideDistance:" << std::endl << sideDistance << std::endl;

        Eigen::Vector2i cellGrid(cell.cast<int>() / _fuseCells);
        bool stop = false;

        while (!stop)
        {
            if (sideDistance[0] < sideDistance[1])
            {
                sideDistance[0] += delta[0];
                cellGrid[0] += step[0];
            }
            else
            {
                sideDistance[1] += delta[1];
                cellGrid[1] += step[1];
            }

            if (((cellGrid * _fuseCells).cast<float>() - cell).norm() * map.info.resolution > range)
                break;


            const Eigen::Vector2i cellMap(cellGrid * _fuseCells);

            for (unsigned int i = 0; i < _fuseCells; ++i)
            {
                for (unsigned int j = 0; j < _fuseCells; ++j)
                {
                    if (map.data[(cellMap[1] + i) * map.info.width + cellMap[0] + j])
                        stop = true;
                }
            }

            if (!stop)
                _data[cellGrid[1]][cellGrid[0]].inspected = true;
        }
    }

    std::cout << std::endl << std::endl;
}
