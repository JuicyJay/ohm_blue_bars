/*
 * PathAnalyserbase.h
 *
 *  Created on: 24.10.2014
 *      Author: m1ch1
 */

#ifndef _PATHANALYSERBASE_H_
#define _PATHANALYSERBASE_H_

#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

namespace analyser
{

typedef struct {
   double linear;
   double angular;
} diff_scale;

typedef struct {
   Vector3d position;
   Vector3d orientation;
} pose;

class PathAnalyser_base
{
public:
   /**
    * @brief constructor
    */
   PathAnalyser_base() {};

   /**
    * @brief destructor
    */
   virtual ~PathAnalyser_base();

   /**
    * @brief set new path
    *
    * @param path -> to analyse incrementelly
    *
    * @return void
    */
   void setPath(std::vector<analyser::pose> path) : _path(path) { }

   /**
    * @param void
    *
    * @return diffcale whitch have to be controled after
    */
   virtual analyser::diff_scale analyse(analyser::pose current_pose) = 0;

   /**
    * @todo write func
    * @return
    */
   static Vector3d quaternion_to_orientationVec(Quaternion<double> q) { return q * Vector3d(1,0,0); }

protected: //dataelements
   std::vector<analyser::pose> _path;  ///< current path to analyse

protected:  //functions
   inline Vector3d currentTarget() { return _path[0]; }
   inline void clearCurrentTarget() { _path.erase(_path.begin()); }

};

} /* namespace analyser */

#endif /* _PATHANALYSERBASE_H_ */
