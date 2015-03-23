/*
 * BasicAnalyser.cpp
 *
 *  Created on: 23.03.2015
 *      Author: m1ch1
 */

#include "BasicAnalyser.h"

namespace analyser
{

BasicAnalyser::BasicAnalyser(std::string config_file) : PathAnalyser_base()
{
   tinyxml2::XMLDocument config;
   config.LoadFile(config_file.c_str());
   if(config.ErrorID() != 0)
   {
      std::cerr << "unable to open XML config file " << config_file << ", will exit now ..." << std::endl;
      exit(EXIT_FAILURE);
   }
   try {
      _target_radius      = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("target_radius")->GetText());
      _target_radius_last = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("target_radius_last")->GetText());
      _cos_pwr_n          = boost::lexical_cast<unsigned int>(config.FirstChildElement("config")->FirstChildElement("cos_pwr_n")->GetText());
      _cos_fac_n          = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("cos_fac_n")->GetText());
      _ang_reached_range  = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("ang_reached_range")->GetText());
      _end_approach       = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("end_approach")->GetText());
   } catch (boost::bad_lexical_cast& e)
   {
      std::cerr << "Error at parsing XML config file" << config_file << " : " << e.what() << std::endl;
      std::cerr << "will exit now...." << std::endl;
      exit(EXIT_FAILURE);
   }
   _reachedLastPose = false;
   _curr_target_radius = _target_radius;
}

BasicAnalyser::~BasicAnalyser()
{
   /* nothing to do */
}

analyser::diff_scale BasicAnalyser::analyse(analyser::pose current_pose)
{
   analyser::diff_scale diff_scale;
   diff_scale.angular = 0;
   diff_scale.linear = 0;

   if(_path.size() == 0)
   {//path with length 0 given... do nothing... return 0:
      this->local_reachedFinalGoal(true);
      return diff_scale;
   }

   if(this->isReachedFinalGoal())
   {
      _reachedLastPose = false;
      return diff_scale;
   }

   Vector3d ori = current_pose.orientation;
   Vector3d pos = current_pose.position;
   //set z to 0, its just a 2d analyser
   pos.z() = 0;

   Vector3d p = this->currentGoal().position - pos;   //get target Vector from pos
   p.z() = 0;

   while(!_reachedLastPose && (p.norm() < _curr_target_radius))
   {
      this->nextGoal();


      p = this->currentGoal().position - pos;
      p.z() = 0;


      if(this->isLastGoal())
      {
         _curr_target_radius = _target_radius_last;
      }

      if(this->isLastGoal() && (p.norm() < _target_radius_last))
      {
         _reachedLastPose = true;
         break;
      }
      //else if(this->isLastGoal())
      //{
      //   this->local_reachedFinalGoal(true);
      //   diff_scale.angular = 0;
      //   diff_scale.linear = 0;
      //   return diff_scale;
      //}
   }

   if(_reachedLastPose)
   {
      //if orientation is nan than dont rotate .... just exit
      if(!this->isDoEndRotate())
      {
         this->local_reachedFinalGoal(true);
         diff_scale.angular = 0;
         diff_scale.linear = 0;
         return diff_scale;
      }

      p = this->currentGoal().orientation;
      p.z() = 0;

   }

   int direction = this->getDirection(p, ori);

   //get scalfactor angular
   double diff_max = M_PI_2;
   double tmp_diff = ::acos(ori.dot(p) / (ori.norm() * p.norm()));
   //reached last pose if arrived last goal
   if(_reachedLastPose && std::abs(tmp_diff) < _ang_reached_range)
   {
      diff_scale.angular = 0;
      this->local_reachedFinalGoal(true);
   }
   else
      diff_scale.angular = (tmp_diff / diff_max) * direction; //scale between -1..1

   //scale factor depending on angle
   double lin_scale_angle = 0;
   //sclae factor depending on distance to last pose
   double lin_scale_dist = 0;

   //compute lin scale
   if(_reachedLastPose)
   {
      diff_scale.linear = 0;
      this->setDistToCurrentGoal(0);
   }
   else
   {
      lin_scale_angle = this->getLinScaleFactor_ang_n(diff_scale.angular);

      if((this->getPathLengthRest() + p.norm()) < _end_approach)
      {
         double tmp = this->getPathLengthRest() + p.norm();
         lin_scale_dist = this->getLinFactor_dist(tmp < 0 ? 0 : tmp);
      }
      else
         lin_scale_dist = 1;

      diff_scale.linear = lin_scale_dist * lin_scale_angle;
      this->setDistToCurrentGoal(p.norm());
   }
   return diff_scale;
}

double BasicAnalyser::scaleFnk_cos_n(unsigned int cos_pwr, double cos_factor, double value)
{
   //cos_pwr must be >= 2
   if(cos_pwr < 2)
      cos_pwr = 2;

   double tmp = 0;
   if(std::abs(value) > M_PI_2 / cos_factor)
   {
      tmp = 0;
   }
   else
   {
      tmp = ::cos(cos_factor * value);
      for (unsigned int i = 0; i < cos_pwr; ++i)
      {
         tmp *= tmp;
      }
   }
   return tmp;
}

int BasicAnalyser::getDirection(Vector3d p, Vector3d ori)
{
   //build cross product to estimate angular direction
   Vector3d directionVec = ori.cross(p);
   double tmp_dir = directionVec(2);
   if(tmp_dir > 0)
      return 1;
   else
      return -1;
}

double BasicAnalyser::getLinFactor_dist(double distance)
{
   //scale factor depending of target distance
   return distance > 1 ? 1 : distance;
}

double BasicAnalyser::getLinScaleFactor_ang_n(double angDiff_scale)
{
   return this->scaleFnk_cos_n(_cos_pwr_n, _cos_fac_n, angDiff_scale);
}

void BasicAnalyser::local_reachedFinalGoal(bool state)
{
   this->setReachedFinalGoal(state);
   _curr_target_radius = _target_radius;
}

} /* namespace analyser */
