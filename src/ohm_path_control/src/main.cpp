#include <ros/ros.h>


#include "PathControl.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ohm_path_control_node");
    ros::NodeHandle nh("~");

    PathControl node;
    node.start(200);

}
