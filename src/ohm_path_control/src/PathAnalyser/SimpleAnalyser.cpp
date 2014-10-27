/*
 * SimpleAnalyser.cpp
 *
 *  Created on: 25.10.2014
 *      Author: m1ch1
 */

#include "SimpleAnalyser.h"

namespace analyser
{

SimpleAnalyser::SimpleAnalyser()
{

}

SimpleAnalyser::~SimpleAnalyser()
{

}

analyser::diff_scale SimpleAnalyser::analyse(analyser::pose current_pose)
{
   analyser::diff_scale diff_scale;
   int direction = 0;

   Vector3d ori = current_pose.orientation;
   Vector3d pos = current_pose.position;
   //set z to 0, its just a 2d analyser
   pos(2) = 0;

   Vector3d p = this->currentTarget() - pos;   //get target Vector from pose

   //build cross product to estimate angular direction
   Vector3d directionVec = ori.cross(p);
   double tmp_dir = directionVec(2);
   int direction = std::abs(tmp_dir) / tmp_dir;

   //get scalfactor angular
   double angular_max = 2 * p.norm();
   diff_scale.angular = direction * ((p - ori).norm() / angular_max);

   //get scalefactor linear
   ///@todo
   diff_scale.linear = 0;

   return diff_scale;
}

} /* namespace controller */
