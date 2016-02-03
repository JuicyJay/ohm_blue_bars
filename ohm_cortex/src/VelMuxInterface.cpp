/*
 * VelMuxInterface.cpp
 *
 *  Created on: 31.10.2014
 *      Author: chris
 */

#include <topic_tools/MuxSelect.h>
#include "VelMuxInterface.h"

namespace autonohm {

static autonohm::VelMuxInterface* _velMuxInterface;

VelMuxInterface* VelMuxInterface::getInstance(void)
{
   if(!_velMuxInterface) _velMuxInterface = new VelMuxInterface();
   return _velMuxInterface;
}

void VelMuxInterface::setNodeHandle(ros::NodeHandle* nh)
{
   _nh = nh;
   this->init();
}

bool VelMuxInterface::activateTeleoperatedVelocity(void)
{
   // check for initialization
   if(!_initFlag) {
      ROS_ERROR("VelMuxInterface not initialized. Please call method init()");
      return false;
   }

   return (this->setHeadTopic(_mux_head_tele_topic) &&
           this->setVelTopic( _mux_vel_tele_topic));
}

bool VelMuxInterface::activateAutonomousVelocity(void)
{
   // check for initialization
   if(!_initFlag) {
      ROS_ERROR("VelMuxInterface not initialized. Please call method init()");
      return false;
   }

   return (this->setHeadTopic(_mux_head_explore_topic) &&
           this->setVelTopic( _mux_vel_explore_topic));
}

void VelMuxInterface::init(void)
{
   std::string mux_vel_select_topic;
   std::string mux_head_select_topic;

   ros::NodeHandle private_nh("~");

   // mutliplexer for velocity
   private_nh.param("mux_select_topic",      mux_vel_select_topic,      std::string("vel/mux/select"));
   private_nh.param("mux_vel_tele_topic",    _mux_vel_tele_topic,       std::string("vel/telop"));
   private_nh.param("mux_vel_explore_topic", _mux_vel_explore_topic,    std::string("vel/explore"));

   // multiplexer for sensor head
   private_nh.param("head_select_topic",      mux_head_select_topic,    std::string("head/mux/select"));
   private_nh.param("head_vel_tele_topic",    _mux_head_tele_topic,     std::string("head/telop"));
   private_nh.param("head_vel_explore_topic", _mux_head_explore_topic,  std::string("head/explore"));

   // init service clients
   _vel_mux_srv  = _nh->serviceClient<topic_tools::MuxSelect>(mux_vel_select_topic);
   _head_mux_srv = _nh->serviceClient<topic_tools::MuxSelect>(mux_head_select_topic);


   _initFlag     = true;
}

bool VelMuxInterface::setVelTopic(std::string topic)
{
   /*
    * Multiplexer for velocity
    */
   topic_tools::MuxSelect srv;
   srv.request.topic = topic;

   if(_vel_mux_srv.call(srv)) return true;
   else                       ROS_ERROR("Failed to call service");
}

bool VelMuxInterface::setHeadTopic(std::string topic)
{
   /*
    * Multiplexer for velocity
    */
   topic_tools::MuxSelect srv;
   srv.request.topic = topic;

   if(_head_mux_srv.call(srv)) return true;
   else                        ROS_ERROR("Failed to call service");

   return false;
}

VelMuxInterface::VelMuxInterface()
{
   _nh       = NULL;
   _initFlag = false;
}

VelMuxInterface::~VelMuxInterface()
{
   if(_velMuxInterface) delete _velMuxInterface;
}

VelMuxInterface::VelMuxInterface(VelMuxInterface& vmi)
{
   _nh       = NULL;
   _initFlag = false;
}

} /* namespace autonohm */
