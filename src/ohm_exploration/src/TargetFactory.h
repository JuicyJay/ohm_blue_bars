/***********************************************************************************************************
 * Class TargetFactory:
 *
 *  Created on: 26.03.2015
 *      Author: Christian Merkl
 *      E-Mail: christian.merkl@th-nuernberg.de
 *     Licence: BSD
 *
 ***********************************************************************************************************/
#ifndef ___TARGET_FACTORY_H___
#define ___TARGET_FACTORY_H___

#include "Target.h"
#include "Wall.h"

#include <vector>

class TargetFactory
{
public:
    TargetFactory(void) { }

    void create(const std::vector<Wall>& walls);

    const std::vector<Target>& targets(void) const { return _targets; }

private:
    std::vector<Target> _targets;
};

#endif
