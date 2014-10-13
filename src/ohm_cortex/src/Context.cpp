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

Context::Context(void)
{
  _currentState = NULL;
}

Context::Context(Context& c)
{
  _currentState = NULL;
}

Context::~Context(void)
{
  if(_context) delete _context;
}

Context* Context::getInstance(void)
{
  if(_context==NULL) _context = new Context();

  return _context;
}

void Context::setState(IState* state)
{
  _currentState = state;
}

void Context::process(void)
{
  if(_currentState) _currentState->process();
}

} /* namespace autonohm */
