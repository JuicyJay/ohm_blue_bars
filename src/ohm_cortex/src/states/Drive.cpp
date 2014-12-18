/*
 * Drive.cpp
 *
 *  Created on: 18.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#include "Drive.h"

namespace autonohm {

Drive::Drive(const geometry_msgs::Pose& target)
    : _nh(autonohm::Context::getInstance()->getNodeHandle())
{

}

Drive::~Drive(void)
{

}

void Drive::process(void)
{

}

} // end namespace autonohm
