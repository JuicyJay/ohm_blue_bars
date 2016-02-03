/*
 * Inspekt.h
 *
 *  Created on: 28.01.2015
 *      Author: knueppl
 */

#ifndef OHM_CORTEX_SRC_STATES_INSPECT_H_
#define OHM_CORTEX_SRC_STATES_INSPECT_H_

#include "../IState.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
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

private:
    ros::NodeHandle* _nh;
    ros::Publisher _state_pub;

    ros::Publisher _pubDirection;
    ros::ServiceClient _srvVictimStack;
    ros::ServiceClient _srvVictimControl;
    ros::ServiceClient _srvHeadMode;
    ros::ServiceClient _srvVictimToWorldControl;

    ros::Time _stamp;
    bool _foundVictim;
    geometry_msgs::Quaternion _orientation;

    static float s_inspectionTime;
};

} /* namespace autonohm */

#endif /* OHM_CORTEX_SRC_STATES_INSPEKT_H_ */
