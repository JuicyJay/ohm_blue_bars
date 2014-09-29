/*
 * Context.cpp
 *
 *  Created on: 29.09.2014
 *      Author: mayst
 */

#include "Context.h"
#include <cstddef>

namespace autonohm
{

static autonohm::Context* _context;

Context::Context()
{
  _currentState = NULL;
}

Context::~Context()
{
  if(_context)
    delete _context;
}

Context* Context::getInstance()
{
  if(_context==NULL)
  {
    _context = new Context();
  }
  return _context;
}

void Context::setState(IState* state)
{
  _currentState = state;
}

void Context::process()
{
  if(_currentState)
    _currentState->process();
}

} /* namespace autonohm */
