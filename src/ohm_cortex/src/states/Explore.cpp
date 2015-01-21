/*
 * Explore.cpp
 *
 *  Created on: 14.10.2014
 *      Author: chris
 *    Modified: Knueppl (Christian Merkl)
 */

#include "Explore.h"

#include <ohm_exploration/GetTarget.h>
#include <ohm_exploration/MarkTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <ohm_sensor_head/Mode.h>

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
    _srvGetTarget  = _nh->serviceClient<ohm_exploration::GetTarget >("/exploration/get_target" );
    _srvMarkTarget = _nh->serviceClient<ohm_exploration::MarkTarget>("/exploration/mark_target");
    _srvTrigger    = _nh->serviceClient<std_srvs::Empty>("/exploration/wall_finder/trigger");
    _srvHeadMode   = _nh->serviceClient<ohm_sensor_head::Mode>("mode");
    _pubTarget = _nh->advertise<geometry_msgs::PoseStamped>("/goal/target", 2);
    _pubDirection = _nh->advertise<geometry_msgs::PoseStamped>("goal/sensor_head", 2);
}


Explore::~Explore(void)
{

}

void Explore::process(void)
{
    TargetStack* stack = TargetStack::instance();

    /* If the stack is empty fill it. */
    if (stack->isEmpty())
    {
        ROS_INFO("Target stack is empty. Will ask the target-stack-node for new targets.");

        ohm_exploration::GetTarget service;
        service.request.id = -1;

        if (!_srvGetTarget.call(service))
        {
            ROS_INFO("No targets available. Will try to find new one in the map.");
            std_srvs::Empty service;

            if (!_srvTrigger.call(service))
            {
                ROS_ERROR("Can't trigger the wall-finder-node. Will kill myself.");
                Context::getInstance()->setState(new Init);
                delete this;
                return;
            }

            ROS_INFO("The wall-finder-node is be triggered. Will recall meself now.");
            Context::getInstance()->setState(new Explore);
            delete this;
            return;
        }

        ROS_INFO_STREAM("Received " << service.response.poses.size() << " targets.");
        stack->setTargets(service.response.id, service.response.poses);
    }


    /* Publish a target to the navigation stuff. */
    geometry_msgs::PoseStamped target;
    static unsigned int seq = 0;

    target.header.seq = ++seq;
    target.header.stamp = ros::Time::now();
    target.header.frame_id = "map";
    target.pose = stack->target();

    _pubTarget.publish(target);

    stack->drop();

    geometry_msgs::PoseStamped direction;

    direction.header = target.header;
    direction.pose = target.pose;

    _pubDirection.publish(direction);


    if (stack->isEmpty())
    {
        ohm_exploration::MarkTarget service;
        service.request.id = stack->id();

        if (!_srvMarkTarget.call(service))
        {
            ROS_ERROR_STREAM("Can't not mark target " << stack->id() << " as be inspected. Will kill myself.");
            Context::getInstance()->setState(new Init);
            delete this;
            return;
        }
    }

    ROS_INFO("Leave state Explore.");

    Context::getInstance()->setState(new Drive(target.pose));
    delete this;
}

} /* namespace autonohm */
