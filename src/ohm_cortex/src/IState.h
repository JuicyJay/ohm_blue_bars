/*
 * IState.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef ISTATE_H_
#define ISTATE_H_

namespace autonohm
{

class IState
{
public:

  virtual ~IState(){};

  virtual void process() = 0;
};

} /* namespace autonohm */

#endif /* ISTATE_H_ */
