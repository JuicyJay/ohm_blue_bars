/*
 * LookAtTarget.h
 *
 *  Created on: 20.03.2015
 *      Author: knueppl
 */

#ifndef OHM_CORTEX_SRC_STATES_LOOK_TO_TARGET_H_
#define OHM_CORTEX_SRC_STATES_LOOK_TO_TARGET_H_

#include "../IState.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/QuaternionStamped.h>

namespace autonohm {

class LookAtTarget : public IState
{
public:

    LookAtTarget(const geometry_msgs::Quaternion& orientation);
    virtual ~LookAtTarget(void);

    virtual void process(void);

private:

    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;
    ros::Publisher _pubDirection;
    ros::ServiceClient _srvHeadMode;

    ros::Time _stamp;
    const geometry_msgs::Quaternion _orientation;
};

} // end namespace autonohm

#endif
