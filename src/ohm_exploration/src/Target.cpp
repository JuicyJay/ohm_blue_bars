#include "Target.h"

unsigned int Target::s_id = 0;

Target::Target(void)
    : _id(-1),
      _inspected(false),
      _valid(false)
{

}

Target::Target(const Pose& pose)
    : _id(-1),
      _inspected(false),
      _pose(pose),
      _valid(true)
{

}

void Target::takeId(void)
{
    _id = ++s_id;
}
