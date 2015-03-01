/************************************************************************************************************
 * Class Target:
 *
 *  Created on: 01.12.2014
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ************************************************************************************************************/
#ifndef ___TARGET_H___
#define ___TARGET_H___

#include "Pose.h"
#include "Wall.h"

class Target
{
public:
    //! Constructs the target from a wall.
    Target(const Wall& wall);

    inline unsigned int id(void) const { return _id; }
    inline const std::vector<Pose>& poses(void) const { return _poses; }
    inline void setInspected(const bool inspected = true) { _inspected = inspected; }
    inline bool inspected(void) const { return _inspected; }

private:
    unsigned int _id;
    bool _inspected;
    std::vector<Pose> _poses;
};

#endif
