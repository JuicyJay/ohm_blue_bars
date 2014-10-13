#include "LeastSquare.h"

#include <cmath>

#include <Eigen/LU>

void LeastSquare::estimateLine(const PointVector& points, Line& line)
{
    if (!points.size())
    {
        line.m = 0.0f;
        line.t = 0.0f;
        return;
    }

    Eigen::Vector2f avg(0.0f, 0.0f);

    for (PointVector::const_iterator point(points.begin()); point < points.end(); ++point)
        avg += point->cast<float>();

    avg /= static_cast<float>(points.size());
    float a = 0.0f;
    float b = 0.0f;

    for (PointVector::const_iterator point(points.begin()); point < points.end(); ++point)
    {
        a += (static_cast<float>(point->x()) - avg.x()) * (static_cast<float>(point->y()) - avg.y());
        b += (static_cast<float>(point->x()) - avg.x()) * (static_cast<float>(point->x()) - avg.x());
    }

    a /= static_cast<float>(points.size());
    b /= static_cast<float>(points.size());
    line.m = a / b;
    line.t = avg.y() - line.m * avg.x();
}
