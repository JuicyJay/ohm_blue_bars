/*
 * StatePong.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef STATEPONG_H_
#define STATEPONG_H_

#include "../IState.h"

namespace autonohm
{

class StatePong: public IState
{
public:
  StatePong(void);
  virtual ~StatePong(void);
  void process(void);
private:
   ros::NodeHandle* _nh;
   ros::Publisher   _state_pub;
};

} /* namespace autonohm */

#endif /* STATEPONG_H_ */
