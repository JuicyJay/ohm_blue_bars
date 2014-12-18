/*
 * TargetStack.cpp
 *
 *  Created on: 17.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#include "TargetStack.h"

TargetStack* TargetStack::_instance = 0;

TargetStack* TargetStack::instance(void)
{
    if (!_instance)
        _instance = new TargetStack;

    return _instance;
}

TargetStack::TargetStack(void)
    : _id(0)
{

}
