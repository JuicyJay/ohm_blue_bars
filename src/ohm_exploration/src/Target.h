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
    inline void setDistanceFromOrigin(const float value) { _distanceFromOrigin = value; }
    inline float distanceFromOrigin(void) const { return _distanceFromOrigin; }

    void takeId(void);

    inline bool operator<(const Target& target) const { return _distance < target._distance; }

private:
    unsigned int _id;
    bool _inspected;
    Pose _pose;
    bool _valid;
    float _distance;
    float _distanceFromOrigin;

    static unsigned int s_id;
};

inline std::ostream& operator<<(std::ostream& os, const Target& target)
{
    os << "Target: id = " << target.id() << ", inspected = " << target.inspected() << ", distance = "
       << target.distance() << ", distance from origin = " << target.distanceFromOrigin();

    return os;
}

#endif
