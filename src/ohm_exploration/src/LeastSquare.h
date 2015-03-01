/************************************************************************************************************
 * Class LeastSquare:
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
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
