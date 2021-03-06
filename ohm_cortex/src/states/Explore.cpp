/*
 * Explore.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 *    Modified: Knueppl (Christian Merkl)
 */

#include "Explore.h"
#include "FrontierExplore.h"

#include <ohm_autonomy/GetTarget.h>
#include <ohm_autonomy/MarkTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>

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

    ROS_INFO("Explore: get target from stack.");
    /* If the stack is empty fill it. */
    if (!stack->getTarget(pose, id))
    {
        ROS_INFO("No targets available.");
        Context::getInstance()->setState(new FrontierExplore(1));
        delete this;
        return;
    }


    /* Publish a target to the navigation stuff. */
    ohm_autonomy::MarkTarget service;
    service.request.id = id;

    ROS_INFO("Explore: mark the target.");
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
