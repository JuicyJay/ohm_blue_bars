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

/**
 * @class   Context
 * @author  Stefan May
 * @date    2014-10-13
 *
 * @brief   Context to manage state machine. Designed as singleton pattern
 */
class Context
{
public:
   /**
    * Function to get instance for singleton pattern
    * @return
    */
  static Context* getInstance();
  /**
   * Default destructor
   */
  virtual ~Context(void);
  /**
   * Function to set state
   * @param state
   */
  void setState(IState* state);
  /**
   * Function for processing
   */
  void process(void);

private:
  /**
   * Private constructor for singleton pattern
   */
  Context();
  /**
   * Private copy constructor for singleton pattern
   * @param c
   */
  Context(Context &c);

  autonohm::IState* _currentState;
};

} /* namespace autonohm */

#endif /* CONTEXT_H_ */
