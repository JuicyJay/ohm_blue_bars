/*
 * Explore.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 *    Modified: Knueppl (Christian Merkl)
 */

#include "Explore.h"

#include <ohm_autonomy/GetTarget.h>
#include <ohm_autonomy/MarkTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <ohm_sensor_head/Mode.h>

#include <Eigen/Geometry>

#include "../Context.h"
#include "TargetStack.h"
#include "Init.h"
#include "Drive.h"

namespace autonohm {

Explore::Explore(void)
    : _nh(autonohm::Context::getInstance()->getNodeHandle())
{
    ROS_INFO("New state is Explore.");

    _state_pub = _nh->advertise<std_msgs::String>("state", 1);

    std_msgs::String msg;
    msg.data = "explore";
    _state_pub.publish(msg);


    /* Communication with the target stack node. */
    _srvTrigger    = _nh->serviceClient<std_srvs::Empty>("/georg/exploration/wall_finder/trigger");
    _srvMarkTarget = _nh->serviceClient<ohm_autonomy::MarkTarget>("/georg/exploration/mark_target");
    _srvNextSection = _nh->serviceClient<std_srvs::Empty>("/georg/mission_planner/trigger/next_section");
}


Explore::~Explore(void)
{

}

void Explore::process(void)
{
    TargetStack* stack = TargetStack::instance();
    geometry_msgs::Pose pose;
    unsigned int id = 0;

    /* If the stack is empty fill it. */
    if (!stack->getTarget(pose, id))
    {
        ROS_INFO("No targets available.");
        std_srvs::Empty service;
        return;
	/*
        if (!_srvTrigger.call(service))
        {
            ROS_ERROR("Can't trigger the wall-finder-node. Will kill myself.");
            Context::getInstance()->setState(new Init);
	    ::exit(1); // HACK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            delete this;
            return;
        }

	if (!_srvNextSection.call(service))
	  {
	    ROS_ERROR("Can't trigger mission planner.");
            Context::getInstance()->setState(new Init);
            delete this;
            return;
	  }

        ROS_INFO("The wall-finder-node is be triggered. Will recall meself now.");
        Context::getInstance()->setState(new Explore);
        delete this;
        return;
	*/
    }


    /* Publish a target to the navigation stuff. */
    ohm_autonomy::MarkTarget service;
    service.request.id = id;

    if (!_srvMarkTarget.call(service))
    {
        ROS_ERROR_STREAM("Can't not mark target " << id << " as be inspected. Will kill myself.");
        Context::getInstance()->setState(new Init);
        delete this;
        return;
    }


    ROS_INFO("Leave state Explore.");
    Context::getInstance()->setState(new Drive(pose.position, pose.orientation));
    delete this;
}

} /* namespace autonohm */
