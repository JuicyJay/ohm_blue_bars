/*
 * IState.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef ISTATE_H_
#define ISTATE_H_


/**
 * @namespace  autonohm
 */
namespace autonohm
{

/**
 * @class   IState
 * @author  Stefan May
 * @date    2014-10-13
 */
class IState
{
public:
  /**
   * Default destructor
   */
  virtual ~IState(void){};

  virtual void process(void) = 0;
};

} /* namespace autonohm */

#endif /* ISTATE_H_ */
