/**
* @file   Line.h
* @author Christian Merkl
*
*/
#ifndef ___LINE_H___
#define ___LINE_H___

#include <Eigen/Core>

#include <ostream>

struct Line
{
    Line(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2)
        : m((p2.x() - p1.x()) / (p2.y() - p1.y())),
          t(p1.y() - m * p1.x())
    {

    }
    Line(const float mValue = 0.0f, const float tValue = 0.0f)
        : m(mValue),
          t(tValue)
    {

    }

    float m;
    float t;
};

inline std::ostream& operator<<(std::ostream& os, const Line& line)
{
    os << "Line(m = " << line.m << ", t = " << line.t << ")";
    return os;
}

#endif
