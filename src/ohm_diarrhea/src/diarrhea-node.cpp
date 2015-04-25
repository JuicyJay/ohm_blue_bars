#include <ros/ros.h>

#include <ohm_common/Obstacle.h>

#include <sstream>

#include "GetTransformation.h"

const float minDistance = 0.1f;
const float radius = 0.15f;
const float stopDistance = 10.0f;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diarrhea_node");
    GetTransformation gt;
    ros::NodeHandle nh;
    ros::Publisher pubMrHanky = nh.advertise<ohm_common::Obstacle>("/georg/path_plan/add_anti_obstacle", 10);

    ros::Rate rate(20);
    Eigen::Vector3f lastPosition(0.0f, 0.0f, 0.0f);
    unsigned int id = 0;

    while (ros::ok())
    {
        gt.lookUpTransform("map", "simon/base_footprint");

        if ((gt.position() - lastPosition).norm() > minDistance)
        {
            ohm_common::Obstacle mrHanky;
            std::stringstream ss;
            ss << ++id;

            lastPosition = gt.position();

            mrHanky.name.data   = ss.str();
            mrHanky.rect.x      = lastPosition.x() - radius * 0.5f;
            mrHanky.rect.y      = lastPosition.y() - radius * 0.5f;
            mrHanky.rect.width  = radius;
            mrHanky.rect.height = radius;

            pubMrHanky.publish(mrHanky);
        }

        if ((gt.position() - lastPosition).norm() > stopDistance)
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }
}
