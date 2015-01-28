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
    _srvGetTarget  = _nh->serviceClient<ohm_exploration::GetTarget >("/georg/exploration/get_target" );
    _srvMarkTarget = _nh->serviceClient<ohm_exploration::MarkTarget>("/georg/exploration/mark_target");
    _srvTrigger    = _nh->serviceClient<std_srvs::Empty>("/georg/exploration/wall_finder/trigger");
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
    geometry_msgs::Pose target(stack->target());
    stack->drop();

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
//    Context::getInstance()->setState(new Drive(target));
    Context::getInstance()->setState(new Drive(target.position, target.orientation));
    delete this;
}

} /* namespace autonohm */
