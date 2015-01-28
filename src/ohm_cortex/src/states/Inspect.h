/*
 * Inspekt.h
 *
 *  Created on: 28.01.2015
 *      Author: knueppl
 */

#ifndef OHM_CORTEX_SRC_STATES_INSPEKT_H_
#define OHM_CORTEX_SRC_STATES_INSPEKT_H_

#include "../IState.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/QuaternionStamped.h>

/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Inspekt
 * @author  Christian Merkl
 * @date    28.01.2015
 */
class Inspect : public IState
{
public:
    Inspect(const geometry_msgs::Quaternion& orientation);
    virtual ~Inspect(void);

    virtual void process(void);

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;
    ros::ServiceClient _srvHeadMode;
    ros::Publisher _pubDirection;

    ros::Time _stamp;
    const geometry_msgs::Quaternion _orientation;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_INSPEKT_H_ */
