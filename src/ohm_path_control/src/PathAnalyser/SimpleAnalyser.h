/*
 * SimpleAnalyser.h
 *
 *  Created on: 25.10.2014
 *      Author: m1ch1
 */

#ifndef PATHANALYSER_SIMPLEANALYSER_H_
#define PATHANALYSER_SIMPLEANALYSER_H_

#include "PathAnalyser_base.h"
#include <cmath>
#include <iostream>
#include <string>
#include <boost/lexical_cast.hpp>
#include <tinyxml2.h>

using namespace Eigen;

namespace analyser
{

class SimpleAnalyser: public analyser::PathAnalyser_base
{
public:
   SimpleAnalyser(std::string config_file);
   virtual ~SimpleAnalyser();

   analyser::diff_scale analyse(analyser::pose current_pose);

private: //functions
   /**
    * @brief contains the transfere function with a cos(mx)^(2..n)
    *
    * @param cos_pwr    -> power of cos(mx) must be >= 2
    * @param cos_factor -> factor of x in cos
    * @param value      -> value to transfers
    * @return  scalefactor 0..1
    */
   double scaleFnk_cos_n(unsigned int cos_pwr, double cos_factor, double value);

   int getDirection(Vector3d p, Vector3d ori);

   double getLinFactor_dist(Vector3d p);

   double getLinScaleFactor_ang_first(double angDiff_scale);

   double getLinScaleFactor_ang_n(double angDiff_scale);

private: //dataelements
   //analyser parameter
   double _target_radius;
   double _target_ratius_last;
   unsigned int _cos_pwr_first;
   double _cos_fac_first;
   unsigned int _cos_pwr_n;
   double _cos_fac_n;
   double _ang_reached_range;

};

} /* namespace controller */

#endif /* PATHANALYSER_SIMPLEANALYSER_H_ */
