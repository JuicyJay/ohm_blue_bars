/**
* @file   Line.h
* @author Christian Merkl
*
*/
#ifndef ___LINE_H___
#define ___LINE_H___

#include <Eigen/Core>

#include <ostream>
#include <cmath>

class Line
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Line(const Eigen::Vector2f& start, const Eigen::Vector2f& end)
        : _m((end.x() - start.x()) / (end.y() - start.y())),
          _t(start.y() - _m * start.x()),
          _r((end - start).normalized()),
          _n(_r.y(), -_r.x()),
          _p1(start),
          _p2(end)
    {

    }

    /* Generates two points for the x values 0.0f and 1.0f. */
    Line(const float mValue = 0.0f, const float tValue = 0.0f)
        : _m(mValue),
          _t(tValue),
          _p1(0.0f, tValue),
          _p2(1.0f, mValue + tValue)
    {
        _r = _p2 - _p1;
        _r = _r.normalized();
        _n = Eigen::Vector2f(_r.y(), -_r.x());
    }

    Line(const Line& line)
    : _m(line._m),
      _t(line._t),
      _r(line._r),
      _n(line._n),
      _p1(line._p1),
      _p2(line._p2)
    {

    }

    inline float m(void) const { return _m; }
    inline float t(void) const { return _t; }
    inline const Eigen::Vector2f& n(void) const { return _n; }
//    inline Eigen::Vector2f n(const Eigen::Vector2i p) const
    inline const Eigen::Vector2f& r(void) const { return _r; }
    inline const Eigen::Vector2f& p1(void) const { return _p1; }
    inline const Eigen::Vector2f& p2(void) const { return _p2; }

    inline float distance(const Eigen::Vector2i& p) const { return std::abs(_n.dot(p.cast<float>() - _p1)); }
    inline float distance(const Eigen::Vector2f& p) const { return std::abs(_n.dot(p - _p1)); }
    inline float x(const float yValue) const { return (yValue - _t) / _m; }
    inline float y(const float xValue) const { return xValue * _m + _t; }

    Line& operator=(const Line& line)
    {
        _m = line._m;
        _t = line._t;
        _r = line._r;
        _n = line._n;
        _p1 = line._p1;
        _p2 = line._p2;

        return *this;
    }

private:
    float _m;
    float _t;
    Eigen::Vector2f _r;
    Eigen::Vector2f _n;
    Eigen::Vector2f _p1;
    Eigen::Vector2f _p2;
};

inline std::ostream& operator<<(std::ostream& os, const Line& line)
{
    os << "Line(m = " << line.m() << ", t = " << line.t() << ", r = (" << line.r().x() << " " << line.r().y()
       << "))";

    return os;
}

#endif
