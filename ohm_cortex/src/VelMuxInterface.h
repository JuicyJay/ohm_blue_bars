/*
 * VelMuxInterface.h
 *
 *  Created on: 31.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_VELMUXINTERFACE_H_
#define OHM_CORTEX_SRC_VELMUXINTERFACE_H_

#include <ros/ros.h>

namespace autonohm {

/**
 * @class   VelMuxInterface
 * @author  Christian Pfitzner
 * @date    2014-10-30
 *
 * @brief   Interface to access vel mux interface
 */
class VelMuxInterface
{
public:
   /**
    * Function to return pointer on instance
    * @return
    */
   static VelMuxInterface* getInstance(void);
   /**
    * Default destructor
    */
   virtual ~VelMuxInterface();

   // SETTERS
   /**
    * Function to set node hanlde
    * @param nh
    */
   void setNodeHandle(ros::NodeHandle* nh);

   // GETTERS


   // PROCESSING
   /**
    * Function to activate teleoperated driving
    * @return  true if everything is ok
    */
   bool activateTeleoperatedVelocity(void);
   /**
    * Function to activate autonomous driving
    * @return  true if everything is ok
    */
   bool activateAutonomousVelocity(void);

   /**
    * Init function to be called after setting of node handle
    */
   void init(void);

private:
   /**
    * Function to activate topic in vel mux
    * @param topic
    * @return
    */
   bool setVelTopic(std::string topic);
   /**
    * Function to activate topic in head mux
    * @param topic
    * @return
    */
   bool setHeadTopic(std::string topic);


   // SINGLETON PATTERN
   /**
    * Private constructor for singleton pattern
    */
   VelMuxInterface(void);
   /**
    * Private copy constructor
    * @param vmi
    */
   VelMuxInterface(VelMuxInterface &vmi);

   ros::NodeHandle*     _nh;                       //!< pointer on ros node handle

   ros::ServiceClient   _vel_mux_srv;              //!< service client to switch mux
   ros::ServiceClient   _head_mux_srv;             //!< service client to switch mux

   std::string          _mux_vel_tele_topic;       //!< service client topic for teloperated vel
   std::string          _mux_vel_explore_topic;    //!< service client topic for autonomous vel
   std::string          _mux_head_tele_topic;      //!< service client topic for teleoperated head moving
   std::string          _mux_head_explore_topic;   //!< service client topic for autonomous head moving

   bool                 _initFlag;                 //!< initialization flag; true if initialized
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_VELMUXINTERFACE_H_ */
