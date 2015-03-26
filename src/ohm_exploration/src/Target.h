/***********************************************************************************************************
 * Class Target:
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ***********************************************************************************************************/
#ifndef ___TARGET_H___
#define ___TARGET_H___

#include "Pose.h"

class Target
{
public:
    Target(void);
    Target(const Pose& pose);

    inline unsigned int id(void) const { return _id; }
    inline const Pose& pose(void) const { return _pose; }
    inline bool valid(void) const { return _valid; }
    inline void setInspected(const bool inspected = true) { _inspected = inspected; }
    inline bool inspected(void) const { return _inspected; }
    inline void setDistance(const float value) { _distance = value; }
    inline float distance(void) const { return _distance; }

    void takeId(void);

    inline bool operator<(const Target& target) const { return _distance < target._distance; }

private:
    unsigned int _id;
    bool _inspected;
    Pose _pose;
    bool _valid;
    float _distance;

    static unsigned int s_id;
};

#endif
