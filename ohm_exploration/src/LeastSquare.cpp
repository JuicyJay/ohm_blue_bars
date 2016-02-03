#include "LeastSquare.h"

#include <cmath>
#include <iostream>

#include <Eigen/LU>

void LeastSquare::estimateLine(const PointVector& points, Line& line)
{
    if (points.size() < 2)
    {
        line = Line();
        return;
    }

    Eigen::Vector2d avg(0.0, 0.0);

    for (PointVector::const_iterator point(points.begin()); point < points.end(); ++point)
    {
        avg += point->cast<double>();
    }

    avg /= points.size();

    double a = 0.0;
    double b = 0.0;

    for (PointVector::const_iterator point(points.begin()); point < points.end(); ++point)
    {
        a += (point->x() - avg.x()) * (point->y() - avg.y());
        b += (point->x() - avg.x()) * (point->x() - avg.x());
    }

    a /= points.size();
    b /= points.size();

    const float m = a / b;
    const float t = avg.y() - m * avg.x();
    line = Line(m, t);
}
