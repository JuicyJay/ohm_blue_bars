/*
 * StatePing.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef STATEPING_H_
#define STATEPING_H_

#include "IState.h"

namespace autonohm
{

class StatePing: public IState
{
public:
  StatePing();
  virtual ~StatePing();
  void process();
};

} /* namespace autonohm */

#endif /* STATEPING_H_ */
