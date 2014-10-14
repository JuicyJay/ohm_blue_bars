/*
 * Init.h
 *
 *  Created on: 14.10.2014
 *      Author: chris
 */

#ifndef OHM_CORTEX_SRC_STATES_INIT_H_
#define OHM_CORTEX_SRC_STATES_INIT_H_


#include "../IState.h"

/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Init
 * @author  Christian Pfitzner
 * @date    2014-10-14
 *
 * @brief   State for initialization of hard and software
 */
class Init : public IState
{
public:
   /**
    * Default constructor
    */
   Init(void);
   /**
    * Default destructor
    */
   virtual ~Init(void);
   /**
    * Function for processing
    */
   void process(void);

private:
   ros::NodeHandle*  _nh;           //!< pointer on node handle
   ros::Publisher    _state_pub;    //!< state publisher
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_INIT_H_ */
