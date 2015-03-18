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
#include <ohm_perception/Victim.h>
/**
 * @namespace autonohm
 */
namespace autonohm {

/**
 * @class   Inspekt
 * @author  Christian Merkl
 * @date    11.03.2015
 */
class Inspect : public IState
{
public:
    Inspect(const geometry_msgs::Quaternion& orientation);
    virtual ~Inspect(void);

    virtual void process(void);

    void callbackVictim(const ohm_perception::Victim& vicitm);

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;
    ros::Publisher _pubDirection;
    ros::Subscriber _subVictim;
    ros::ServiceClient _srvHeadMode;
    ros::ServiceClient _srvVictimControl;

    ros::Time _stamp;
    const geometry_msgs::Quaternion _orientation;
    bool _foundVictim;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_INSPEKT_H_ */
