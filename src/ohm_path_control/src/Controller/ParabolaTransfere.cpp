/*
 * ParabolaTransfere.cpp
 *
 *  Created on: 04.11.2014
 *      Author: m1ch1
 */

#include "ParabolaTransfere.h"

namespace controller
{

ParabolaTransfere::ParabolaTransfere(std::string config_file)
{
   tinyxml2::XMLDocument config;
   config.LoadFile(config_file.c_str());
   if(config.ErrorID() != 0)
   {
      std::cerr << "unable to open XML config file " << config_file << ", will exit now ..." << std::endl;
      exit(EXIT_FAILURE);
   }
   try {
      _max_vel_lin = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("vel_max_lin")->GetText());
      _max_vel_ang = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("vel_max_ang")->GetText());
      _parabola_lin_fac = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("parabola_lin_fac")->GetText());
      _parabola_ang_fac = boost::lexical_cast<double>(config.FirstChildElement("config")->FirstChildElement("parabola_ang_fac")->GetText());
   } catch (boost::bad_lexical_cast& e)
   {
      std::cerr << "Error at parsing XML config file" << config_file << " : " << e.what() << std::endl;
      std::cerr << "will exit now...." << std::endl;
      exit(EXIT_FAILURE);
   }
}

ParabolaTransfere::~ParabolaTransfere()
{
}

controller::velocity ParabolaTransfere::control(double linear, double angular)
{
   controller::velocity vel;

   vel.angular = this->parabola(_parabola_ang_fac, angular,_max_vel_ang);
   vel.linear = this->parabola(_parabola_lin_fac, linear, _max_vel_lin);

   return vel;
}

double ParabolaTransfere::parabola(double scale, double value, double max_value)
{
   max_value = std::abs(max_value);
   int sng = value > 0 ? 1 : -1;
   double tmp = scale * std::abs(value) * value;
   return std::abs(tmp) > max_value ? (max_value * sng) : tmp;
}

} /* namespace controller */
