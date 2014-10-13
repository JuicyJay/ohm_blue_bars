#include "Ransac.h"
#include "LeastSquare.h"

#include <ctime>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <unistd.h>

#include <Eigen/LU>

Ransac::Ransac(void)
    : _maxIterations(100),
      _epsilon(10),
      _minPoints(10)
{
    std::srand(static_cast<unsigned int>(std::time(0)));
}

bool Ransac::estimateLines(std::vector<Line>& lines, const unsigned int maxNumberOfLines)
{
    PointVector storage = _points;
    Line line;

    while (this->estimateLine(line))
    {
        lines.push_back(line);
        PointVector outliers;

        for (PointVector::const_iterator point(_points.begin()); point < _points.end(); ++point)
        {
            if (std::abs(static_cast<float>(point->x()) * line.m + line.t - static_cast<float>(point->y())) >=
                _epsilon)
            {
                outliers.push_back(*point);
            }
        }

        if (lines.size() >= maxNumberOfLines)
            break;

        _points = outliers;
    }

    return lines.size() ? true : false;
}

bool Ransac::estimateLine(Line& line)
{
    if (_points.size() < 2)
    {
        std::cout << __PRETTY_FUNCTION__ << ": no input points set." << std::endl;
        return false;
    }

    std::vector<Line> lines;
    int mostPoints = -1;
    unsigned int best = 0;

    for (unsigned int i = 0; i < _maxIterations; i++)
    {
        PointVector model;
        this->getRandomlyPoints(model, 2);

        if (model.size() != 2)
            return false;

        if (model[0].x() == model[1].x())
            continue;

        const float m = static_cast<float>(model[0].y() - model[1].y()) /
            static_cast<float>(model[0].x() - model[1].x());
        const float t = model[0].y() - m * model[0].x();


        /* debug output */
//        std::cout << "RANSAC iteration " << i << std::endl;
//        std::cout << "-----------------------" << std::endl;
//        std::cout << "Take point " << model[0] << " and " << model[1] << std::endl;
//        std::cout << "Model parameter: m = " << m << " t = " << t << std::endl;

        PointVector linePoints;

        for (PointVector::const_iterator point(_points.begin()); point < _points.end(); ++point)
            if (std::abs(static_cast<float>(point->x()) * m + t - static_cast<float>(point->y())) < _epsilon)
                linePoints.push_back(*point);

//        std::cout << "found " << linePoints.size() << " points." << std::endl;

        if (linePoints.size() >= _minPoints)
        {
            Line line;
            LeastSquare::estimateLine(linePoints, line);
            lines.push_back(line);

            if (static_cast<int>(linePoints.size()) > mostPoints)
            {
                mostPoints = linePoints.size();
                best = i;
            }
        }
    }

    if (mostPoints < 0)
        return false;

    line = lines[best];
    return true;
}

bool Ransac::estimateWall(Wall& wall)
{
    if (_points.size() < 2)
    {
        std::cout << __PRETTY_FUNCTION__ << ": no input points set." << std::endl;
        return false;
    }

    std::vector<Wall> walls;
    int mostPoints = -1;
    unsigned int best = 0;

    for (unsigned int i = 0; i < _maxIterations; i++)
    {
        PointVector model;
        this->getRandomlyPoints(model, 2);

        if (model.size() != 2)
            return false;

        if (model[0].x() == model[1].x())
            continue;

        const float m = static_cast<float>(model[0].y() - model[1].y()) /
            static_cast<float>(model[0].x() - model[1].x());
        const float t = model[0].y() - m * model[0].x();


        /* debug output */
        std::cout << "RANSAC iteration " << i << std::endl;
        std::cout << "-----------------------" << std::endl;
        std::cout << "Take point (" << model[0].x() << ", " << model[0].y() << ") and ("
                  << model[1].x() << ", " << model[1].y() << ")" << std::endl;
        std::cout << "Model parameter: m = " << m << " t = " << t << std::endl;

        PointVector linePoints;

        for (PointVector::const_iterator point(_points.begin()); point < _points.end(); ++point)
            if (std::abs(static_cast<float>(point->x()) * m + t - static_cast<float>(point->y())) < _epsilon)
                linePoints.push_back(*point);

//        std::cout << "found " << linePoints.size() << " points." << std::endl;

        if (linePoints.size() >= _minPoints)
        {
            walls.push_back(Wall(linePoints));;

            if (static_cast<int>(linePoints.size()) > mostPoints)
            {
                mostPoints = linePoints.size();
                best = i;
            }
        }
    }

    if (mostPoints < 0)
        return false;

    wall = walls[best];
    return true;
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

bool Ransac::checkIfAllPointsFarEnough(const PointVector& points, const int distance)
{
    for (unsigned int i = 0; i < points.size(); i++)
    {
        for (unsigned int j = 0; j < points.size(); j++)
        {
            if (i == j)
                continue;

            if (std::abs(points[i].x() - points[j].x()) < distance)
                return false;
        }
    }

    return true;
}
