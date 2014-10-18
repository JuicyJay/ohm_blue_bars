#include "Ransac.h"
#include "LeastSquare.h"

#include <ctime>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <unistd.h>

#include <Eigen/LU>
#include <Eigen/Geometry>

Ransac::Ransac(void)
    : _maxIterations(100),
      _epsilon(10),
      _minPoints(10)
{
    std::srand(static_cast<unsigned int>(std::time(0)));
}

bool Ransac::estimateWall(Wall& wall)
{
    if (_points.size() < 2)
    {
        std::cout << __PRETTY_FUNCTION__ << ": no input points set." << std::endl;
        return false;
    }

    PointVector points;
    int mostPoints = -1;

    for (unsigned int i = 0; i < _maxIterations; i++)
    {
        PointVector modelPoints;
        this->getRandomlyPoints(modelPoints, 2);

        if (modelPoints.size() != 2)
            return false;

        if (modelPoints[0].x() == modelPoints[1].x())
            continue;

        Line model(modelPoints[0].cast<float>(), modelPoints[1].cast<float>());

        /* debug output */
//        std::cout << "RANSAC iteration " << i << std::endl;
//        std::cout << "-----------------------" << std::endl;
//        std::cout << "Take point (" << model.p1().x() << ", " << model.p1().y() << ") and ("
//                  << model.p2().x() << ", " << model.p2().y() << ")" << std::endl;
//        std::cout << "Model parameter: " << model << std::endl;

        PointVector linePoints;

        this->getPointsByLine(model, linePoints);
        LeastSquare::estimateLine(linePoints, model);
        linePoints.clear();
        this->getPointsByLine(model, linePoints);
        LeastSquare::estimateLine(linePoints, model);
        linePoints.clear();
        this->getPointsByLine(model, linePoints);

        if (linePoints.size() >= _minPoints)
        {
            if (static_cast<int>(linePoints.size()) > mostPoints)
            {
                mostPoints = linePoints.size();
                points = linePoints;
            }
        }
    }

    if (mostPoints < 0)
        return false;

    wall = Wall(points);
    return true;
}

void Ransac::getPointsByLine(const Line& line, PointVector& points)
{
    for (PointVector::const_iterator point(_points.begin()); point < _points.end(); ++point)
        if (line.distance(*point) < _epsilon)
            points.push_back(*point);
}


void Ransac::getRandomlyPoints(PointVector& points, const unsigned int numberOf)
{
    if (!_points.size())
    {
        std::cout << __PRETTY_FUNCTION__ << ": no input points set." << std::endl;
        return;
    }

    do
    {
        points.clear();

        for (unsigned int i = 0; i < numberOf; i++)
        {
            points.push_back(_points[std::rand() % _points.size()]);
        }
    }
    while (!this->checkIfAllPointsDifferent(points));
}

bool Ransac::checkIfAllPointsDifferent(const PointVector& points)
{
    for (unsigned int i = 0; i < points.size(); i++)
    {
        for (unsigned int j = 0; j < points.size(); j++)
        {
            if (i == j)
                continue;

            if (points[i] == points[j])
                return false;
        }
    }

    return true;
}
