/*
 * Context.h
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#ifndef CONTEXT_H_
#define CONTEXT_H_


#include "IState.h"
#include <ros/ros.h>

#include <std_srvs/Empty.h>


/**
 * @namespace autonohm
 */
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
  static Context* getInstance(void);
  /**
   * Default destructor
   */
  virtual ~Context(void);


  // SETTERS
  /**
   * Function to set state
   * @param state
   */
  void setState(IState* state);
  /**
   * Function to set node handle to context
   * @param nh
   */

  void setNodeHandle(ros::NodeHandle* nh);


  // GETTERS
  /**
   * Function to get current state
   * @return
   */
  IState* getState(void) const;

  /**
   * Function to get node handle
   * @return
   */
  ros::NodeHandle* getNodeHandle(void) const;


  // PROCESSING
  /**
   * Function for processing
   */
  void process(void);

private:
  /**
   * Private constructor for singleton pattern
   */
  Context(void);
  /**
   * Private copy constructor for singleton pattern
   * @param c
   */
  Context(Context &c);

  // MEMBERS
  autonohm::IState*     _currentState;
  ros::NodeHandle*      _nh;


};

} /* namespace autonohm */

#endif /* CONTEXT_H_ */
