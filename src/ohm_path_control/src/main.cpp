#include <ros/ros.h>


#include "PathControl.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "NodeName");
    ros::NodeHandle nh("~");

    PathControl node;
    node.start(10);

}
