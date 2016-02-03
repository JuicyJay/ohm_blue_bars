/*
 * StatePing.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef STATEPING_H_
#define STATEPING_H_

#include "../IState.h"

namespace autonohm {


class StatePing: public IState
{
public:

   StatePing(void);
   virtual ~StatePing(void);
   void process(void);

private:
   ros::NodeHandle* _nh;

   ros::Publisher   _state_pub;
};

} /* namespace autonohm */

#endif /* STATEPING_H_ */
