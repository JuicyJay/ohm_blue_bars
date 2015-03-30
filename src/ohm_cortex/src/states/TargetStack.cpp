/*
 * TargetStack.cpp
 *
 *  Created on: 17.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#include "TargetStack.h"

#include <ohm_autonomy/GetTarget.h>

TargetStack* TargetStack::_instance = 0;

TargetStack* TargetStack::instance(void)
{
    if (!_instance)
        _instance = new TargetStack;

    return _instance;
}

TargetStack::TargetStack(void)
{
    _srvGetTarget  = _nh.serviceClient<ohm_autonomy::GetTarget >("/georg/exploration/get_target" );
}

bool TargetStack::getTarget(geometry_msgs::Pose& pose, unsigned int& id)
{
    ohm_autonomy::GetTarget service;
    service.request.id = ohm_autonomy::GetTarget::Request::NEXT;

    if (!_srvGetTarget.call(service))
    {
        return false;
    }

    pose = service.response.pose;
    id = service.response.id;
    return true;
}
