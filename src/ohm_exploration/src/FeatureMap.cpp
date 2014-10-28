#include "FeatureMap.h"
#include "Ray.h"

#include <iostream>

#include <ros/console.h>

FeatureMap::FeatureMap(void)
    : _wallDepth(5),
      _width(0),
      _height(0)
{

}

void FeatureMap::setMap(const nav_msgs::OccupancyGrid& map)
{
    if (!map.info.width || !map.info.height)
    {
        ROS_ERROR_STREAM("FeatureMap::setMap(): map isn't valid. Width and height must not 0.");
        return;
    }

    _data.resize(map.info.height);
    _width = map.info.width;
    _height = map.info.height;

    for (unsigned int row = 0; row < _data.size(); ++row)
        _data[row].resize(map.info.width);

    this->updateMap(map);
}

void FeatureMap::updateMap(const nav_msgs::OccupancyGrid& map)
{
    /* Check the changeovers in the map in positive x direction. */
    for (unsigned int row = 1; row < _data.size() - 1; ++row)
    {
        const unsigned int offset = map.info.width * row;
        bool changeover = false;
        unsigned int depth = 0;
        int8_t last = map.data[offset];

        for (unsigned int col = 1; col < _data[row].size(); ++col)
        {
            const int8_t current = map.data[offset + col];

            if (!last && current > 0)
            {
                if (map.data[map.info.width * (row - 1) + col] > 0 &&
                    map.data[map.info.width * (row + 1) + col] > 0)
                {
                    changeover = true;
                    _data[row][col].orientation |= Wall::Left;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row][col].orientation |= Wall::Left;
            }
            else if (changeover && current < 0)
            {
                changeover = false;
            }

            last = current;
        }
    }


    /* Check the changeovers in the map in negative x direction. */
    for (unsigned int row = map.info.height - 2; row > 0; --row)
    {
        const unsigned int offset = map.info.width * row;
        int8_t last = map.data[offset + map.info.width - 1];
        unsigned int depth = 0;
        bool changeover = false;

        for (unsigned int col = map.info.width - 2; col > 0; --col)
        {
            const int8_t current = map.data[offset + col];

            if (!last && current > 0)
            {
                if (map.data[map.info.width * (row - 1) + col] > 0 &&
                    map.data[map.info.width * (row + 1) + col] > 0)
                {
                    changeover = true;
                    _data[row][col].orientation |= Wall::Right;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row][col].orientation |= Wall::Right;
            }
            else if (changeover && current < 0)
            {
                changeover = false;
            }

            last = current;
        }
    }


    /* Check the changeover in the map in positve y direction. */
    for (unsigned int col = 1; col < map.info.width - 1; ++col)
    {
        int8_t last = map.data[col];
        unsigned int depth = 0;
        bool changeover = false;

        for (unsigned int row = 1; row < map.info.height - 1; ++row)
        {
            const int8_t current = map.data[map.info.width * row + col];

            if (!last && current > 0)
            {
                if (map.data[map.info.width * row + (col - 1)] > 0 &&
                    map.data[map.info.width * row + (col + 1)] > 0)
                {
                    changeover = true;
                    _data[row][col].orientation |= Wall::Up;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row][col].orientation |= Wall::Up;
            }
            else if (changeover && current < 0)
            {
                changeover = false;
            }

            last = current;
        }
    }


    /* Check the changeover in the map in negative y direction. */
    for (unsigned int col = 1; col < map.info.width - 2; ++col)
    {
        int8_t last = map.data[map.info.width * (map.info.height - 1) + col];
        unsigned int depth = 0;
        bool changeover = false;

        for (unsigned int row = map.info.height - 2; row > 0; --row)
        {
            const int8_t current = map.data[map.info.width * row + col];

            if (!last && current > 0)
            {
                if (map.data[map.info.width * row + (col - 1)] > 0 &&
                    map.data[map.info.width * row + (col + 1)] > 0)
                {
                    changeover = true;
                    _data[row][col].orientation |= Wall::Down;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row][col].orientation |= Wall::Down;
            }
            else if (changeover && current < 0)
            {
                changeover = false;
            }

            last = current;
        }

    }
}

void FeatureMap::exportPoints(PointVector& points, const Wall::Orientation orientation)
{
    for (unsigned int y = 0; y < _height; ++y)
        for (unsigned int x = 0; x < _width; ++x)
            if (_data[y][x].orientation & orientation  && !(_data[y][x].saw & orientation))
                points.push_back(Eigen::Vector2i(x, y));
}

void FeatureMap::markWalls(const std::vector<Wall>& walls)
{
    for (std::vector<Wall>::const_iterator wall(walls.begin()); wall < walls.end(); ++wall)
    {
        const float thickness = 0.2f / wall->resolution();
        Eigen::Vector2f start(wall->center() - wall->model().r() * wall->length() * 0.5f);

        for (Ray x(start - wall->model().n() * thickness, wall->model().r(), wall->length()); x.next();)
        {
            for (Ray y(x.position().cast<float>(), wall->model().n(), thickness * 2.0f); y.next();)
            {
                if (y.position().x() >= _width || y.position().y() >= _height)
                {
                    std::cout << "out of range: x = " << y.position().x()
                              << " y = " << y.position().y() << std::endl;
                    std::cout << "start: x = " << start.x() << " y = " << start.y() << std::endl;
                    std::cout << "center: x = " << wall->center().x() << " y = " << wall->center().y() << std::endl;
                    std::cout << "model: " << wall->model() << std::endl;
                    std::cout << "points:" << std::endl;

                    for (unsigned int i = 0; i < wall->points().size(); ++i)
                        std::cout << "(" << wall->points()[i].x() << " " << wall->points()[i].y() << ")" << std::endl;
                    break;
//                    return;
                }
                else
                {
                    _data[y.position().y()][y.position().x()].saw |= wall->orientation();
                }
            }
        }
    }
}

void FeatureMap::paintImage(cv::Mat& image, const FeatureCell must)
{
    image.create(_height, _width, CV_8UC1);

    for (int row = 0; row < image.rows; ++row)
        for (int col = 0; col < image.cols; ++col)
        {
//            image.at<uint8_t>(row, col) = _data[row][col] == must ? 0xff : 0x00;
            image.at<uint8_t>(row, col) = _data[row][col].saw > 0 ? 0xff : 0x00;
//            std::cout << static_cast<int>(_data[row][col].saw) << " ";
        }
    std::cout << std::endl;
}

