/*
 * Straight2D.cpp
 *
 *  Created on: Jul 9, 2018
 *      Author: ninahetterich
 */

#include "Straight2D.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <typeinfo>

// intercept point between two lines -> used in latest code version
Straight2D::Straight2D(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2):
_start(point1),
_dir(point2 - point1)
{
	//std::cout << __PRETTY_FUNCTION__ << " start\n" << _start << std::endl;
	//std::cout << __PRETTY_FUNCTION__ << " dir\n " << _dir <<std::endl;
}

Straight2D::~Straight2D()
{
}

Eigen::Vector2d Straight2D::cut(const Straight2D& second)const
{
	Eigen::Matrix2d mat;
	mat.col(0) = _dir;
	mat.col(1) = -second._dir;
	Eigen::Vector2d c = second._start - _start;
	Eigen::Vector2d var = mat.inverse() * c;
	return _start + var(0) * _dir;
}

// distance between a line and a point, not used in latest code version
double Straight2D::distPointLine(const Eigen::Vector2d& point)
{
	double dist = 0.0;
	Eigen::Vector2d var = (point - _start);
	Eigen::Vector3d cross;
	Eigen::Vector3d var3d(var(0), var(1), 0.0);
	Eigen::Vector3d dir3d(_dir(0), _dir(1), 0.0);
	cross = var3d.cross(dir3d);
	double abs = cross.norm();
	dist = abs / _dir.norm();
	return dist;
}


// distance between two lines, not used in latest code version
double Straight2D::distLineLine(const Straight2D& second)const

{
	double dist = 0.0;

	Eigen::Vector2d differenz= _start - second._start;
	double zahl=(differenz(0)*differenz(0)) +(differenz(1)*differenz(1));
	dist = sqrt(zahl);

	return dist;
}


// angle between two lines, not used in latest code version
double Straight2D::angle(const Straight2D& second)const
{
	double deg = 0.0;
	double skal = std::abs(_dir.dot(second._dir)); // dot product

	double l1 = _dir.norm(); // magnitude of the vectors
	double l2 = second._dir.norm();

	double product = l1 * l2;
	double k = (skal / product);

	double rad = std::acos(k); // angle in rad
	deg =  (360.0 / (2.0 * M_PI)) * rad;// angle in degree

	return deg + 90.0;
}



