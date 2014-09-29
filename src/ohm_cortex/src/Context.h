/*
 * Context.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_

#include "IState.h"

namespace autonohm
{

class Context
{
public:
  static Context* getInstance();

  virtual ~Context();

  void setState(IState* state);

  void process();

private:

  Context();

  autonohm::IState* _currentState;
};

} /* namespace autonohm */

#endif /* CONTEXT_H_ */
