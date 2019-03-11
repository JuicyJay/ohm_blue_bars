/*
 * Straight2D.h
 *
 *  Created on: Jul 9, 2018
 *      Author: ninahetterich
 */

#ifndef TESTPRGS_EIGEN_GEOMETRIC_STRAIGHT2D_H_
#define TESTPRGS_EIGEN_GEOMETRIC_STRAIGHT2D_H_

#include <Eigen/Dense>

class Straight2D {
public:
	Straight2D(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2);
	virtual ~Straight2D();
	Eigen::Vector2d cut(const Straight2D& second) const;

	double distPointLine(const Eigen::Vector2d& point);
	double distLineLine(const Straight2D& second)const;
	double angle(const Straight2D& second) const;

private:
	Eigen::Vector2d _start;
	Eigen::Vector2d _dir;
};


#endif
