/**
* @file   LeastSquare.h
* @author Christian Merkl
*
*/
#ifndef ___LEAST_SQUARE_H___
#define ___LEAST_SQUARE_H___

#include "Line.h"
#include "Wall.h"

#include <vector>

#include <Eigen/Core>

class LeastSquare
{
public:

    static void estimateLine(const PointVector& points, Line& line);
};

#endif
