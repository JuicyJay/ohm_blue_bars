/*
 * FrontierExplore.h
 *
 *  Created on: 30.01.2015
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_
#define OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_

#include "../IState.h"

/**
 * @namespace  autonohm
 */
namespace autonohm {

/**
 * @class   FrontierExplore
 * @author  Christian Pfitzner
 * @date    2015-01-30
 *
 * @brief   State class for
 */
class FrontierExplore : IState
{
public:
   /**
    * Default constructor
    */
   FrontierExplore(void);
   /**
    * Default destructor
    */
   virtual ~FrontierExplore(void);


   // PROCESSING
   /**
    * Function for processing
    */
   void process(void);

private:
   ros::NodeHandle*     _nh;
   ros::Publisher       _state_pub;

   ros::ServiceClient   _target_srv_client;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_FRONTIEREXPLORE_H_ */
