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
//#define EIGEN_USE_NEW_STDVECTOR
//#include <Eigen/StdVector>

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
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} pose;

class PathAnalyser_base
{
public:
   /**
    * @brief constructor
    */
   PathAnalyser_base() : _currentPose_index(0) {};

   /**
    * @brief destructor
    */
   virtual ~PathAnalyser_base() {}

   /**
    * @brief set new path
    *
    * @param path -> to analyse incrementelly
    *
    * @return void
    */
   void setPath(std::vector<analyser::pose> path) { _path = std::vector<analyser::pose>(path); _currentPose_index = 0;}

   /**
    * @param void
    *
    * @return diffscale whitch have to be controled after
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
   inline analyser::pose currentTarget() { return _path[_currentPose_index]; }
   inline void nextTarget() { if(_currentPose_index < _path.size() - 1) _currentPose_index++; }
   inline unsigned int getCurrentPoseIndex() { return _currentPose_index; }
   inline bool isLastPose() { return _currentPose_index == _path.size() - 1 ? true : false; }
   inline bool isFirstPose() { return _currentPose_index == 0 ? true : false; }

private:
   unsigned int _currentPose_index;

   //for eigen
//public:
   //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} /* namespace analyser */

#endif /* _PATHANALYSERBASE_H_ */
