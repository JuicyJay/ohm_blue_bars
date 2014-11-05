#include "Ray.h"

#include <iostream>

Ray::Ray(const Eigen::Vector2f origin, const Eigen::Vector2f v, const float length)
    : _origin(origin),
      _v(v),
      _length(length),
      _position(origin.cast<int>()),
      _delta(std::sqrt(1 + (_v.y() * _v.y()) / (_v.x() * _v.x())),
             std::sqrt(1 + (_v.x() * _v.x()) / (_v.y() * _v.y())))
{
    if (_v.x() < 0)
    {
        _step.x() = -1;
        _sideDistance.x() = (_origin.x() - static_cast<float>(static_cast<int>(_origin.x()))) * _delta.x();
    }
    else
    {
        _step.x() = 1;
        _sideDistance.x() = (static_cast<float>(static_cast<int>(_origin.x()) + 1) - _origin.x()) * _delta.x();
    }
    if (_v.y() < 0)
    {
        _step.y() = -1;
        _sideDistance.y() = (_origin.y() - static_cast<float>(static_cast<int>(_origin.y()))) * _delta.y();
    }
    else
    {
        _step.y() = 1;
        _sideDistance.y() = (static_cast<float>(static_cast<int>(_origin.y()) + 1) - _origin.y()) * _delta.y();
    }
}

bool Ray::next(void)
{
//    std::cout << __PRETTY_FUNCTION__ << std::endl;

    if (_sideDistance.x() < _sideDistance.y())
    {
        _sideDistance.x() += _delta.x();
        _position.x() += _step.x();
    }
    else
    {
        _sideDistance.y() += _delta.y();
        _position.y() += _step.y();
    }

//    if (std::abs(_position.x()) >= 4000 || std::abs(_position.y()) >= 4000)
//    {
//        std::cout << "origin   = " << _origin.x() << " " << _origin.y() << std::endl;
//        std::cout << "position = " << _position.x() << " " << _position.y() << std::endl;
//        std::cout << "length   = " << _length << std::endl;
//        return false;
//    }

    return (_position.cast<float>() - _origin).norm() < _length;
}
