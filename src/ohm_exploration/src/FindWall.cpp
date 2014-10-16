#include "FindWall.h"

#include <ostream>

#include <opencv2/opencv.hpp>

FindWall::FindWall(void)
{
    _ransac.setEpsilon(1.5f);
    _ransac.setMinimumPoints(30);
    _ransac.setMaxIterations(100);

    cv::namedWindow("debug");
}

void FindWall::setMap(const nav_msgs::OccupancyGrid& map)
{
    _points.clear();
    _featureMap.setMap(map);
    this->exportPoints(map);
    this->buildCluster(map);
    _mapMetaData = map.info;
}

void FindWall::search(std::vector<Wall>& walls)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;

    Wall wall;
    _ransac.setInputPoints(_points);

    while (_ransac.estimateWall(wall))
//    _ransac.estimateWall(wall);
    {
        std::cout << "will work with " << _points.size() << " points." << std::endl;
        wall.setResolution(_mapMetaData.resolution);
        wall.setOrigin(_mapMetaData.origin.position);
        walls.push_back(wall);
        std::cout << wall << std::endl;

        this->removePoints(wall.points());
        _ransac.setInputPoints(_points);
    }
}

void FindWall::exportPoints(const nav_msgs::OccupancyGrid& map)
{
    for (unsigned int row = 0; row < map.info.height; ++row)
    {
        const unsigned int offset = map.info.width * row;

        for (unsigned int col = 0; col < map.info.width; ++col)
//            if (map.data[offset + col] > 0)
            if (_featureMap(col, row).orientation == FeatureCell::Up && map.data[offset + col] > 0)
                _points.push_back(Eigen::Vector2i(col, row));
    }
}

void FindWall::removePoints(const PointVector& points)
{
    std::vector<bool> mask(_points.size(), true);
    PointVector rest;

    for (unsigned int i = 0; i < points.size(); ++i)
    {
        for (unsigned int j = 0; j < _points.size(); ++j)
        {
            mask[j] = mask[j] & (_points[j] != points[i]);
        }
    }

    for (unsigned int i = 0; i < mask.size(); ++i)
        if (mask[i])
            rest.push_back(_points[i]);

    _points = rest;
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
