/*
 * Drive.h
 *
 *  Created on: 14.10.2014
 *      Author: Knueppl (Christian Merkl)
 */
#ifndef ___DRIVE_H___
#define ___DRIVE_H___

#include "../IState.h"

#include <ros/ros.h>

/**
 * @namespace autonohm
 */
namespace autonohm {

class Drive : public IState
{
public:
    Drive(const geometry_msgs::Pose& target);

    virtual ~Drive(void);
    virtual void process(void);

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;
};

} // end namespace autonohm

#endif
