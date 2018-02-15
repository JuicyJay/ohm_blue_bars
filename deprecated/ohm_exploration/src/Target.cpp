#include "Target.h"

unsigned int Target::s_id = 0;

Target::Target(void)
    : _id(-1),
      _inspected(false),
      _valid(false),
      _distance(0.0f),
      _distanceFromOrigin(0.0f)
{

}

Target::Target(const Pose& pose)
    : _id(-1),
      _inspected(false),
      _pose(pose),
      _valid(true),
      _distance(0.0f),
      _distanceFromOrigin(0.0f)
{

}

void Target::takeId(void)
{
    _id = ++s_id;
}
