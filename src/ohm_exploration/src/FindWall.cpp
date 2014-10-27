#include "FindWall.h"

#include <ostream>

#include <opencv2/opencv.hpp>

FindWall::FindWall(void)
    : _points(4)
{
    _ransac.setEpsilon(2.0f);
    _ransac.setMinimumPoints(40);
    _ransac.setMaxIterations(100);

    _orientations.push_back(Wall::Up);
    _orientations.push_back(Wall::Down);
    _orientations.push_back(Wall::Left);
    _orientations.push_back(Wall::Right);
//    cv::namedWindow("debug");
}

void FindWall::setMap(const nav_msgs::OccupancyGrid& map)
{
    if (_featureMap.isNull())
        _featureMap.setMap(map);
    else
        _featureMap.updateMap(map);

    for (unsigned int i = 0; i < _orientations.size(); ++i)
        _featureMap.exportPoints(_points[i], _orientations[i]);

    _mapMetaData = map.info;
}

void FindWall::search(std::vector<Wall>& walls)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    for (unsigned int i = 0; i < _orientations.size(); ++i)
    {
        Wall wall;
        _ransac.setInputPoints(_points[i]);

        while (_ransac.estimateWall(wall))
        {
            std::cout << "will work with " << _points[i].size() << " points." << std::endl;
            std::cout << wall << std::endl;

            if (wall.valid())
            {
                wall.setResolution(_mapMetaData.resolution);
                wall.setOrigin(_mapMetaData.origin.position);
                wall.setOrientation(_orientations[i]);
                walls.push_back(wall);
            }

            this->removePoints(wall.points(), _points[i]);
            _ransac.setInputPoints(_points[i]);
        }
    }

    _featureMap.markWalls(walls);
    /*
    FeatureCell cell;
    cell.orientation = Wall::Up;
    cell.saw = Wall::Up;

    cv::Mat image;

    _featureMap.paintImage(image, cell);
    cv::imshow("debug", image);
    */
}

void FindWall::exportPoints(const nav_msgs::OccupancyGrid& map,
                            PointVector& points,
                            const Wall::Orientation orientation)
{
    for (unsigned int row = 0; row < map.info.height; ++row)
    {
        const unsigned int offset = map.info.width * row;

        for (unsigned int col = 0; col < map.info.width; ++col)
        {
            if (_featureMap(col, row).orientation & orientation)//  &&  map.data[offset + col] > 0)
                points.push_back(Eigen::Vector2i(col, row));
        }
    }
}

 void FindWall::removePoints(const PointVector& remove, PointVector& points)
{
    std::vector<bool> mask(points.size(), true);
    PointVector rest;

    for (unsigned int i = 0; i < remove.size(); ++i)
    {
        for (unsigned int j = 0; j < points.size(); ++j)
        {
            mask[j] = mask[j] & (points[j] != remove[i]);
        }
    }

    for (unsigned int i = 0; i < mask.size(); ++i)
        if (mask[i])
            rest.push_back(points[i]);

    points = rest;
}

void FindWall::buildCluster(const nav_msgs::OccupancyGrid& map)
{
    cv::Mat mask(map.info.height, map.info.width, CV_8UC1);

    for (unsigned int row = 0; row < map.info.height; ++row)
    {
        const unsigned int offset = map.info.width * row;

        for (unsigned int col = 0; col < map.info.width; ++col)
        {
            mask.at<uint8_t>(row, col) = map.data[offset + col] > 0 ? 0xff : 0x00;
        }
    }

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    cv::Mat image(mask.rows, mask.cols, CV_8UC3);
    image = cv::Scalar(0x00);

    for (unsigned int i = 0; i < contours.size(); ++i)
        cv::drawContours(image, contours, i, cv::Scalar(0x00, 0x00, 0xff));

    cv::imshow("debug", image);
}
