
#include "SemiAutonohm.h"

SemiAutonohm::SemiAutonohm() : _rate(0)
{
    _loopRate = 0;

    //rosParam
    ros::NodeHandle privNh("~");
    std::string sub_name_goal;
    std::string sub_name_pose;
    std::string action_name;

    privNh.param("sub_name_goal",sub_name_goal,std::string("/move_base_simple/goal"));
    privNh.param("sub_name_pose",sub_name_pose,std::string("pose"));
    privNh.param("action_name",action_name,std::string("move_to"));

    //init publisher
    //_pub = _nh.advertise<ROS_PACK::MSG>("topicName",1);

    //inti subscriber
    _subGoal = _nh.subscribe(sub_name_goal,1,&SemiAutonohm::subGoal_callback,this);
    _subPose = _nh.subscribe(sub_name_pose,1,&SemiAutonohm::subPose_callback,this);

    //init action
    _ac = new actionlib::SimpleActionClient<ohm_path_plan::MoveToAction>(action_name, true);
}

SemiAutonohm::~SemiAutonohm()
{
    delete _rate;
}

void SemiAutonohm::start(const unsigned int rate)
{
    delete _rate;
    _loopRate = rate;
    _rate = new ros::Rate(_loopRate);

    ROS_INFO("Wait for action server...");
    _ac->waitForServer();
    ROS_INFO("Action server active...");
    this->run();
}

void SemiAutonohm::run()
{
   ros::spin();
//    unsigned int cnt = 0;
//
//    while(ros::ok())
//    {
//        //do stuff;
//
//        //publish data;
//        //_pub.publish(msg);
//
//        ros::spinOnce();
//        _rate->sleep();
//    }
}


void SemiAutonohm::subGoal_callback(const geometry_msgs::PoseStamped& msg)
{
   //stop current path controller;
   ROS_INFO("Transmitt goal to server");
   ohm_path_plan::MoveToGoal goal;
   goal.start = _currentPose;
   goal.end = msg;

   _ac->sendGoal(goal);

}

void SemiAutonohm::subPose_callback(const geometry_msgs::PoseStamped& msg)
{
   _currentPose = msg;
}


//---------------------------------------------------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ohm_semiautonom_node");
    ros::NodeHandle nh("~");

    SemiAutonohm node;
    node.start(10);

}


