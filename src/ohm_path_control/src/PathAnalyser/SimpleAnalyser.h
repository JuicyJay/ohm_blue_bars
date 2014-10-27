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

namespace analyser
{

class SimpleAnalyser: public analyser::PathAnalyser_base
{
public:
   SimpleAnalyser();
   virtual ~SimpleAnalyser();

   analyser::diff_scale analyse(analyser::pose current_pose);
};

} /* namespace controller */

#endif /* PATHANALYSER_SIMPLEANALYSER_H_ */
