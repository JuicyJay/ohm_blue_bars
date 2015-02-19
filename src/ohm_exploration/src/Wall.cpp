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
      _resolution(1.0f),
      _orientation(None),
      _distance(-1.0f)
{
//    std::cout << __PRETTY_FUNCTION__ << std::endl;
//    std::cout << "points = " << _points.size() << std::endl;

//    for (unsigned int i = 0; i < points.size(); ++i)
//        std::cout << "(" << points[i].x() << ", " << points[i].y() << ")   ";

    LeastSquare::estimateLine(_points, _model);

    std::sort(_points.begin(), _points.end(), *this);
    PointVector::const_iterator pointEnd(_points.begin() + 1);

    for (PointVector::const_iterator point(_points.begin()); pointEnd < _points.end(); ++point, ++pointEnd)
    {
        if ((*point - *pointEnd).cast<float>().norm() > 5.0f)
            break;
    }

    if (pointEnd != _points.end())
        _points.resize(pointEnd - _points.begin());

//    std::cout << "points = " << _points.size() << std::endl;
    _valid = _points.size() >= 30;

    if (!_valid)
        return;

    for (PointVector::const_iterator point(_points.begin()); point < _points.end(); ++point)
        _center += point->cast<float>();

    _center /= static_cast<float>(_points.size());

//    std::cout << "valid  = " << _valid << std::endl;

    _length = (_points.back() - _points.front()).cast<float>().norm();
    _valid &= _length >= 30;
//    for (unsigned int i = 0; i < _points.size(); ++i)
//    {
//        std::cout << "(" << _points[i].x() << ", " << _points[i].y() << ")" << std::endl;
//    }
//
//    std::cout << std::endl;
}

Wall::Wall(const ohm_exploration::Wall& wall)
    : _model(Eigen::Vector2f(0.0f, 0.0f), Eigen::Vector2f(wall.v.x, wall.v.y)),
      _id(wall.id),
      _center(Eigen::Vector2f(wall.center.x, wall.center.y)),
      _resolution(1.0f),
//      _orientation(static_cast<Orientation>(wall.orientation)),
      _valid(true),
      _length(wall.length),
      _distance(-1.0f)
{
    this->setOrientation(static_cast<Orientation>(wall.orientation));
}

Wall::Wall(const Wall& wall)
    : _model(wall._model),
      _points(wall._points),
      _id(wall._id),
      _center(wall._center),
      _resolution(wall._resolution),
      _origin(wall._origin),
      _orientation(wall._orientation),
      _valid(wall._valid),
      _length(wall._length),
      _distance(wall._distance)
{

}

bool Wall::operator()(const Eigen::Vector2i& left, const Eigen::Vector2i& right) const
{
    return (left - Eigen::Vector2i(0, _model.t())).cast<float>().norm() <
        (right - Eigen::Vector2i(0, _model.t())).cast<float>().norm();
}

void Wall::setOrientation(const Orientation orientation)
{
    _orientation = orientation;

    // to do: correct normal if they is in wrong direction.
    switch (orientation)
    {
    case Up:
    case Right:
        break;

    case Down:
    case Left:
        _model.flipNormal();
        break;

    default:
        break;
    }
}

visualization_msgs::Marker Wall::getMarkerMessage(void) const
{
    visualization_msgs::Marker marker;

    if (_points.size() < 2)
        return marker;

    Eigen::Vector2f thick((_points.front().cast<float>() - _center).normalized());
    Eigen::Rotation2Df rot(M_PI * 0.5f);
    thick = rot.matrix() * thick;
    thick *= 0.02f;

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
    marker.pose.position.x -= thick.x() * 0.5f;
    marker.pose.position.y -= thick.y() * 0.5f;
    marker.pose.orientation.x = 0.0f;
    marker.pose.orientation.y = 0.0f;
    marker.pose.orientation.z = 0.0f;
    marker.pose.orientation.w = 1.0f;

    switch (_orientation)
    {
    case Up:
        /* red */
        marker.color.r = 198.0f / 256.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        break;

    case Down:
        /* green */
        marker.color.r =  63.0f / 256.0f;
        marker.color.g = 202.0f / 256.0f;
        marker.color.b =  58.0f / 256.0f;
        break;

    case Left:
        /* blue */
        marker.color.r =  63.0f / 256.0f;
        marker.color.g =  84.0f / 256.0f;
        marker.color.b = 221.0f / 256.0f;
        break;

    case Right:
        /* purple */
        marker.color.r = 180.0f / 256.0f;
        marker.color.g =   0.0f / 256.0f;
        marker.color.b = 175.0f / 256.0f;
        break;

    default:
        marker.color.r = 0.8f;
        marker.color.g = 0.8f;
        marker.color.b = 0.8f;
        break;
    };

    marker.color.a = 1.0f;




    /* front side part */
    /* max */
    Eigen::Vector2f min(_points.front().cast<float>() + _model.n() * _model.distance(_points.front()));
    Eigen::Vector2f max(_points.back().cast<float>() + _model.n() * _model.distance(_points.back()));
    geometry_msgs::Point point;

    point.x = max.x() * _resolution;
    point.y = max.y() * _resolution;
    point.z = 0.0f;
    marker.points.push_back(point);

    /* min */
    point.x = min.x() * _resolution;
    point.y = min.y() * _resolution;
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

    Eigen::Vector2f dummy(min * _resolution + thick);
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

ohm_exploration::Wall Wall::getWallMessage(void) const
{
    ohm_exploration::Wall msg;

    msg.id          = _id;
    msg.orientation = _orientation;

    msg.center.x = _center.x() * _resolution + _origin.x;
    msg.center.y = _center.y() * _resolution + _origin.y;
    msg.center.z = _origin.z;

    msg.v.x = _model.r().x();
    msg.v.y = _model.r().y();
    msg.v.z = 0.0f;

    msg.n.x = _model.n().x();
    msg.n.y = _model.n().y();
    msg.n.z = 0.0f;

    msg.length = _length * _resolution;

    return msg;
}
