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
//    for (unsigned int i = 0; i < points.size(); ++i)
//        std::cout << "(" << points[i].x() << ", " << points[i].y() << ")   ";

    LeastSquare::estimateLine(_points, _model);

    for (PointVector::const_iterator point(_points.begin()); point < _points.end(); ++point)
        _center += point->cast<float>();

    _center /= static_cast<float>(_points.size());

    std::sort(_points.begin(), _points.end(), *this);
    PointVector::const_iterator pointEnd(_points.begin() + 1);

    for (PointVector::const_iterator point(_points.begin()); pointEnd < _points.end(); ++point, ++pointEnd)
    {
//        std::cout << "(" << point->x() << ", " << point->y() << ") - (" << pointEnd->x() << ", " << pointEnd->y()
//                  << ") = " << (*point - *pointEnd).cast<float>().norm() << std::endl;

        if ((*point - *pointEnd).cast<float>().norm() > 3)
            break;
    }

    if (pointEnd != _points.end())
        _points.resize(pointEnd - _points.begin());

//    for (unsigned int i = 0; i < _points.size(); ++i)
//    {
//        std::cout << "(" << _points[i].x() << ", " << _points[i].y() << ")" << std::endl;
//    }
//
//    std::cout << std::endl;
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
    if (left.x() == right.x())
        return left.y() < right.y();
    else
        return left.x() < right.x();
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

    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;


    Eigen::Vector2f thick((_points.front().cast<float>() - _center).normalized());
    Eigen::Rotation2Df rot(M_PI * 0.5f);
    thick = rot.matrix() * thick;
    thick *= 0.01f;


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

    marker.points.push_back(marker.points[17]);
    marker.points.push_back(marker.points[0]);
    point = marker.points[0];
    point.z = 0.9f;
    marker.points.push_back(point);


    /* top stuff */
    marker.points.push_back(marker.points[2]);
    marker.points.push_back(marker.points[5]);
    marker.points.push_back(marker.points[13]);

    marker.points.push_back(marker.points[5]);
    marker.points.push_back(marker.points[13]);
    marker.points.push_back(marker.points[17]);


    return marker;
}
