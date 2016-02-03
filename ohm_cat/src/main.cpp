#include <ros/ros.h>


#include "OhmCat.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ohm_cat_node");
    ros::NodeHandle nh("~");

    OhmCat node;
    node.start(50);

}
