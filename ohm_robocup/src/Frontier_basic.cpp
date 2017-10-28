
#include "Frontier_basic.h"

#include <ohm_apps_msgs/NodeControl.h>

Frontier_basic::Frontier_basic(void)
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string string_val;
    double      double_val;
    int         int_val;
    bool        bool_val;

    privNh.param(         "string_val" ,    string_val,   std::string("string"));
    privNh.param<double>( "double_val" ,    double_val,   100.0);
    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
    privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);


    //init publisher
    _pub = _nh.advertise<std_msgs::Bool>("pub_name",1);

    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _subFrontiers = _nh.subscribe("frontiers", 1);
    _subMoveState = _nh.subscribe("rona/state", 2);

    _srv_ctrl_frontier = _nh.serviceClient<ohm_apps_msgs::NodeControl>("frontier/node_control");
}

Frontier_basic::~Frontier_basic()
{

}

void Frontier_basic::start(double duration)
{
   //create timer
   _loopTimer = _nh.createTimer(ros::Duration(duration), &Frontier_basic::loop_callback, this);
   this->run();
}

void Frontier_basic::run()
{
   ros::spin();
}


void Frontier_basic::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!

}

void Frontier_basic::subFrontier_callback(const geometry_msgs::PoseArray& msg)
{

}

void Frontier_basic::subState_callback(const rona_msgs::State& msg)
{

}






// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh("~");

    Frontier_basic node;
    node.start();

}
