/**
* @file   Ransac.h
* @author Christian Merkl
*
*/
#ifndef ___RANSAC_H___
#define ___RANSAC_H___

#include "Line.h"
#include "Wall.h"

#include <vector>

#include <Eigen/Core>

class Ransac
{
public:
    Ransac(void);

    void setInputPoints(const PointVector& points) { _points = points; }
    void setEpsilon(const float epsilon) { _epsilon = epsilon; }
    void setMinimumPoints(const unsigned int points) { _minPoints = points; }
    void setMaxIterations(const unsigned int iterations) { _maxIterations = iterations; }

    bool estimateWall(Wall& wall);

private:
    void getPointsByLine(const Line& line, const float epsilon, PointVector& points);
    void getRandomlyPoints(PointVector& points, const unsigned int numberOf);
    bool checkIfAllPointsDifferent(const PointVector& points);

    PointVector _points;
    unsigned int _maxIterations;
    float _epsilon;
    unsigned int _minPoints;
};

#endif
