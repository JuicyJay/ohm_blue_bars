/*
 * Context.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#include "Context.h"
#include <cstddef>

#include <std_srvs/Empty.h>


namespace autonohm
{

static autonohm::Context* _context;

Context::Context(void)
{
  _currentState = NULL;
  _nh           = NULL;
}

Context::Context(Context& c)
{
  _currentState = NULL;
  _nh           = NULL;
}

Context::~Context(void)
{
  if(_context) delete _context;
}


// GETTERS
Context* Context::getInstance(void)
{
  if(_context==NULL) _context = new Context();

  return _context;
}

IState* Context::getState(void) const
{
   return _currentState;
}

ros::NodeHandle* Context::getNodeHandle(void) const
{
   return _nh;
}


// SETTERS
void Context::setState(IState* state)
{
  _currentState = state;
}

void Context::setNodeHandle(ros::NodeHandle* nh)
{
   _nh = nh;

}


// PROCESSING
void Context::process(void)
{
  if(_currentState) _currentState->process();
}



} /* namespace autonohm */
