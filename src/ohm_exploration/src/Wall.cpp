#include "Wall.h"
#include "LeastSquare.h"

#include <iostream>
#include <algorithm>

#include <Eigen/Geometry>

unsigned int Wall::s_id = 0;

Wall::Wall(const PointVector& points)
    : _points(points),
      _id(++s_id),
      _center(0.0f, 0.0f),
      _resolution(1.0f)
{
    LeastSquare::estimateLine(_points, _model);

    for (PointVector::const_iterator point(_points.begin()); point < _points.end(); ++point)
        _center += point->cast<float>();

    _center /= static_cast<float>(_points.size());

    std::sort(_points.begin(), _points.end(), *this);

//    for (unsigned int i = 0; i < _points.size(); ++i)
//    {
//        std::cout << "(" << points[i].x() << ", " << points[i].y() << ")   ";
//        std::cout << "(" << _points[i].x() << ", " << _points[i].y() << ")" << std::endl;
//    }
}

Wall::Wall(const Wall& wall)
    : _model(wall._model),
      _points(wall._points),
      _id(wall._id),
      _center(wall._center),
      _resolution(wall._resolution),
      _origin(wall._origin)
{

}

bool Wall::operator()(const Eigen::Vector2i& left, const Eigen::Vector2i& right) const
{
    return left.x() < right.x();

    std::cout << __PRETTY_FUNCTION__ << std::endl;

    const Eigen::Vector2f diffLeft ( left.cast<float>() - _center);
    const Eigen::Vector2f diffRight(right.cast<float>() - _center);

    std::cout << "diffLeft:" << std::endl << diffLeft << std::endl;
    std::cout << "diffRight:" << std::endl << diffRight << std::endl;
    std::cout << "length left = " << (diffLeft.x() < 0.0f ? -diffLeft.norm() : diffLeft.norm()) << std::endl;
    std::cout << "length right = " << (diffRight.x() < 0.0f ? -diffRight.norm() : diffRight.norm()) << std::endl;

    return diffLeft.x() < 0.0f ? -diffLeft.norm() : diffLeft.norm() <
        diffRight.x() < 0.0f ? -diffRight.norm() : diffRight.norm();
}

visualization_msgs::Marker Wall::getMarkerMessage(void) const
{
    visualization_msgs::Marker marker;

    if (_points.size() < 2)
        return marker;

    marker.header.frame_id = "map";
    marker.header.stamp    = ros::Time::now();
    marker.ns              = "";
    marker.id              = _id;
    marker.type            = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action          = visualization_msgs::Marker::ADD;

    marker.scale.x = 1.0f;
    marker.scale.y = 1.0f;
    marker.scale.z = 1.0f;

    marker.pose.position = _origin;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
    marker.pose.orientation.w = 1.0f;

    marker.color.r = 0.8f;
    marker.color.g = 0.8f;
    marker.color.b = 0.8f;
    marker.color.a = 1.0f;


    Eigen::Vector2f thick((_points.front().cast<float>() - _center).normalized());
    Eigen::Rotation2Df rot(M_PI * 0.5f);
    thick = rot.matrix() * thick;
    thick *= 0.05f;


    /* front side part */
    /* max */
    geometry_msgs::Point point;
    point.x = _points.back().x() * _resolution;
    point.y = _points.back().y() * _resolution;
    point.z = 0.0f;
    marker.points.push_back(point);

    /* min */
    point.x = _points.front().x() * _resolution;
    point.y = _points.front().y() * _resolution;
    point.z = 0.0f;
    marker.points.push_back(point);

    point.z = 0.9f;
    marker.points.push_back(point);

    marker.points.push_back(marker.points[0]);
    point = marker.points[1];
    point.z = 0.9f;
    marker.points.push_back(point);

    point = marker.points[0];
    point.z = 0.9f;
    marker.points.push_back(point); // 6 items


    /* side part one */
    marker.points.push_back(marker.points[1]);
    marker.points.push_back(marker.points[2]);

    Eigen::Vector2f dummy(_points.front().cast<float>() * _resolution + thick);
    point.x = dummy.x();
    point.y = dummy.y();
    point.z = 0.0f;
    marker.points.push_back(point);

    marker.points.push_back(marker.points[7]);
    marker.points.push_back(marker.points[8]);

    dummy = Eigen::Vector2f(marker.points[2].x, marker.points[2].y) + thick;
    point.x = dummy.x();
    point.y = dummy.y();
    point.z = 0.9f;
    marker.points.push_back(point); // 12 items


    /* back side part */
    marker.points.push_back(marker.points[10]);
    marker.points.push_back(marker.points[11]);

    dummy = Eigen::Vector2f(marker.points[0].x, marker.points[0].y) + thick;
    point.x = dummy.x();
    point.y = dummy.y();
    point.z = 0.0f;
    marker.points.push_back(point);

    marker.points.push_back(marker.points[11]);
    marker.points.push_back(point);
    point.z = 0.9f;
    marker.points.push_back(point); // 18 items


    /* side part two */
    marker.points.push_back(point);
    point.z = 0.0f;
    marker.points.push_back(point);
    marker.points.push_back(marker.points[0]);

    marker.points.push_back(marker.points[20]);
    marker.points.push_back(marker.points[21]);
    point = marker.points[0];
    point.z = 0.9f;
    marker.points.push_back(point);


    return marker;
}
