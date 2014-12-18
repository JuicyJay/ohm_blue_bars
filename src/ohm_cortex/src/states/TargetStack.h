/*
 * TargetStack.h
 *
 *  Created on: 17.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#ifndef ___TARGET_STACK_H___
#define ___TARGET_STACK_H___

#include <vector>

#include <geometry_msgs/Pose.h>

class TargetStack
{
public:
    static TargetStack* instance(void);

    void setTargets(const unsigned int id, const std::vector<geometry_msgs::Pose>& targets)
    {
        _targets = targets;
        _id = id;
    }

    geometry_msgs::Pose target(void) const { return _targets.size() ? _targets.back() : geometry_msgs::Pose(); }
    inline void drop(void) { _targets.pop_back(); }
    inline void clear(void) { _targets.clear(); }
    inline bool isEmpty(void) const { return !_targets.size(); }
    inline unsigned int id(void) const { return _id; }

private:
    TargetStack(void);

    unsigned int _id;
    std::vector<geometry_msgs::Pose> _targets;

    static TargetStack* _instance;
};

#endif
