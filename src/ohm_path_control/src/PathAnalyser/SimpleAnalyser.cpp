/*
 * SimpleAnalyser.cpp
 *
 *  Created on: 25.10.2014
 *      Author: m1ch1
 */

#include "SimpleAnalyser.h"

namespace analyser
{

SimpleAnalyser::SimpleAnalyser(std::string config_file)
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
      _target_ratius_last = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("target_ratius_last")->GetText());
      _cos_pwr_first      = boost::lexical_cast<unsigned int>(config.FirstChildElement("config")->FirstChildElement("cos_pwr_first")->GetText());
      _cos_fac_first      = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("cos_fac_first")->GetText());
      _cos_pwr_n          = boost::lexical_cast<unsigned int>(config.FirstChildElement("config")->FirstChildElement("cos_pwr_n")->GetText());
      _cos_fac_n          = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("cos_fac_n")->GetText());
      _ang_reached_range = 0.1;
   } catch (boost::bad_lexical_cast& e)
   {
      std::cerr << "Error at parsing XML config file" << config_file << " : " << e.what() << std::endl;
      std::cerr << "will exit now...." << std::endl;
      exit(EXIT_FAILURE);
   }
}

SimpleAnalyser::~SimpleAnalyser()
{

}

analyser::diff_scale SimpleAnalyser::analyse(analyser::pose current_pose)
{
   analyser::diff_scale diff_scale;

   Vector3d ori = current_pose.orientation;
   Vector3d pos = current_pose.position;
   //set z to 0, its just a 2d analyser
   pos(2) = 0;

   Vector3d p = this->currentTarget().position - pos;   //get target Vector from pos

   bool reachedLastPose = false;

   //prove Target reached
   if(p.norm() < _target_radius)
   {
      //std::cout << "Reached " << this->getCurrentPoseIndex() << ". target" << std::endl;
      this->nextTarget();
      p = this->currentTarget().position - pos;
      if(this->isLastPose())
      {
         reachedLastPose = true;
         p = this->currentTarget().orientation;
      }
   }

   int direction = this->getDirection(p, ori);

   //get scalfactor angular
   double diff_max = M_PI_2;
   double tmp_diff = ::acos(ori.dot(p) / (ori.norm() * p.norm()));
   if(reachedLastPose && std::abs(tmp_diff) < _ang_reached_range)
      diff_scale.angular = 0;
   else
      diff_scale.angular = (tmp_diff / diff_max) * direction; //scale between -1..1



   //scale factor depending on angle
   double lin_scale_angle = 0;
   //sclae factor depending on distance to last pose
   double lin_scale_dist = 0;

   //compute lin scale
   if(reachedLastPose)
   {
      diff_scale.linear = 0;
   }
   else
   {
      if(this->isFirstPose())
         lin_scale_angle = this->getLinScaleFactor_ang_first(diff_scale.angular);
      else
         lin_scale_angle = this->getLinScaleFactor_ang_n(diff_scale.angular);

      if(this->isLastPose())
         lin_scale_dist = this->getLinFactor_dist(p);
      else
         lin_scale_dist = 1;

      diff_scale.linear = lin_scale_dist * lin_scale_angle;
   }

   return diff_scale;
}

double SimpleAnalyser::scaleFnk_cos_n(unsigned int cos_pwr, double cos_factor, double value)
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

int SimpleAnalyser::getDirection(Vector3d p, Vector3d ori)
{
   //build cross product to estimate angular direction
   Vector3d directionVec = ori.cross(p);
   double tmp_dir = directionVec(2);
   if(tmp_dir > 0)
      return 1;
   else
      return -1;
}

double SimpleAnalyser::getLinFactor_dist(Vector3d p)
{
   //get scalefactor linear
   double dist_target = p.norm();
   //scale factor depending of target distance
   return dist_target > 1 ? 1 : dist_target;
}

double SimpleAnalyser::getLinScaleFactor_ang_first(double angDiff_scale)
{
   return this->scaleFnk_cos_n(_cos_pwr_first, _cos_fac_first, angDiff_scale);
}

double SimpleAnalyser::getLinScaleFactor_ang_n(double angDiff_scale)
{
   return this->scaleFnk_cos_n(_cos_pwr_n, _cos_fac_n, angDiff_scale);
}

} /* namespace controller */
