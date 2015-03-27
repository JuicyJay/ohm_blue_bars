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

void FeatureMap::setMap(const Map& map)
{
    if (map.isNull())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": map isn't valid. Width and height must not 0.");
        return;
    }

    _data.resize(map.height());
    _width  = map.width();
    _height = map.height();

    for (unsigned int row = 0; row < _data.size(); ++row)
        _data[row].resize(map.width());

    this->updateMap(map);
}

void FeatureMap::updateMap(const Map& map)
{
    if (this->isNull())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": feature map is not initialized. Please frist call setMap()"
                         "before updateMap().");
        return;
    }
    if (map.width() != _data[0].size() || map.height() != _data.size())
    {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": map has the wrong size. The size of the map is " <<
                         map.width() << " x " << map.height() << " and of the feature map is " <<
                         _data[0].size() << " x " << _data.size() << ".");
        return;
    }

    /* Check the changeovers in the map in positive x direction. */
    for (unsigned int row = 1; row < map.height() - 1; ++row)
    {
//        const unsigned int offset = map.info.width * row;
        bool changeover = false;
        unsigned int depth = 0;
//        int8_t last = map.data[offset];
        int8_t last = map(0, row);

        for (unsigned int col = 1; col < map.width(); ++col)
        {
//            const int8_t current = map.data[offset + col];
            const int8_t current = map(col, row);

            if (!last && current > 0)
            {
//                if (map.data[map.info.width * (row - 1) + col] > 0 &&
//                    map.data[map.info.width * (row + 1) + col] > 0)
                if (map(col, row - 1) > 0 && map(col, row + 1))
                {
                    changeover = true;
                    _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Left;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Left;
            }
            else if (changeover && current < 0)
            {
                changeover = false;
            }

            last = current;
        }
    }


    /* Check the changeovers in the map in negative x direction. */
    for (unsigned int row = map.height() - 2; row > 0; --row)
    {
//        const unsigned int offset = map.info.width * row;
//        int8_t last = map.data[offset + map.info.width - 1];
        int8_t last = map(map.width() - 1, row);
        unsigned int depth = 0;
        bool changeover = false;

        for (unsigned int col = map.width() - 2; col > 0; --col)
        {
//            const int8_t current = map.data[offset + col];
            const int8_t current = map(col, row);

            if (!last && current > 0)
            {
//                if (map.data[map.info.width * (row - 1) + col] > 0 &&
//                    map.data[map.info.width * (row + 1) + col] > 0)
                if (map(col, row - 1) > 0 && map(col, row + 1))
                {
                    changeover = true;
                    _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Right;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Right;
            }
            else if (changeover && current < 0)
            {
                changeover = false;
            }

            last = current;
        }
    }


    /* Check the changeover in the map in positve y direction. */
    for (unsigned int col = 1; col < map.width() - 1; ++col)
    {
//        int8_t last = map.data[col];
        int8_t last = map(col, 0);
        unsigned int depth = 0;
        bool changeover = false;

        for (unsigned int row = 1; row < map.height() - 1; ++row)
        {
//            const int8_t current = map.data[map.info.width * row + col];
            const int8_t current = map(col, row);

            if (!last && current > 0)
            {
//                if (map.data[map.info.width * row + (col - 1)] > 0 &&
//                    map.data[map.info.width * row + (col + 1)] > 0)
                if (map(col - 1, row) > 0 && map(col + 1, row))
                {
                    changeover = true;
                    _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Up;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Up;
            }
            else if (changeover && current < 0)
            {
                changeover = false;
            }

            last = current;
        }
    }


    /* Check the changeover in the map in negative y direction. */
    for (unsigned int col = 1; col < map.width() - 2; ++col)
    {
//        int8_t last = map.data[map.info.width * (map.info.height - 1) + col];
        int8_t last = map(col, map.height() - 1);
        unsigned int depth = 0;
        bool changeover = false;

        for (unsigned int row = map.height() - 2; row > 0; --row)
        {
//            const int8_t current = map.data[map.info.width * row + col];
            const int8_t current = map(col, row);

            if (!last && current > 0)
            {
//                if (map.data[map.info.width * row + (col - 1)] > 0 &&
//                    map.data[map.info.width * row + (col + 1)] > 0)
                if (map(col - 1, row) > 0 && map(col + 1, row))
                {
                    changeover = true;
                    _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Down;
                    depth = 0;
                }
            }
            else if (changeover && current > 0)
            {
                if (++depth >= _wallDepth)
                    changeover = false;

                _data[row + map.roi().y()][col + map.roi().x()].orientation |= Wall::Down;
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
                if (y.position().x() >= static_cast<int>(_width) ||
                    y.position().y() >= static_cast<int>(_height))
                {
                    /*
                    std::cout << "out of range: x = " << y.position().x()
                              << " y = " << y.position().y() << std::endl;
                    std::cout << "start: x = " << start.x() << " y = " << start.y() << std::endl;
                    std::cout << "center: x = " << wall->center().x() << " y = " << wall->center().y() << std::endl;
                    std::cout << "model: " << wall->model() << std::endl;
                    std::cout << "points:" << std::endl;

                    for (unsigned int i = 0; i < wall->points().size(); ++i)
                        std::cout << "(" << wall->points()[i].x() << " " << wall->points()[i].y() << ")" << std::endl;
                    */
                    break;
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
            image.at<uint8_t>(row, col) = _data[row][col].saw > 0 ? 0xff : 0x00;
}

