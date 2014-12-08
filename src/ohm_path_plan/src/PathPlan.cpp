
#include "PathPlan.h"

PathPlan::PathPlan() :
      _rate(0)
{
    _loopRate = 0;


    //rosParam
    ros::NodeHandle privNh("~");
    std::string pub_name_path;
    std::string sub_name_map;
    std::string action_name;
    std::string frame_id;
    double robot_radius;
    //int int_val;
    
    privNh.param("pub_name_path",pub_name_path,std::string("path"));
    privNh.param("sub_name_map",sub_name_map,std::string("map"));
    privNh.param("action_name",action_name,std::string("move_to"));
    privNh.param("frame_id",frame_id,std::string("map"));
    privNh.param<double>("robot_radius",robot_radius, 0.3);

    _robotRadius = robot_radius;
    _frame_id = frame_id;
    //init publisher
    _pubPath = _nh.advertise<nav_msgs::Path>(pub_name_path,1);

    //inti subscriber
    _subMap = _nh.subscribe(sub_name_map, 1, &PathPlan::subCallback_map, this);

    //init action
    _actionMoveTo = new actionlib::SimpleActionServer<ohm_path_plan::MoveToAction>(_nh, action_name, false);
    _actionMoveTo->registerGoalCallback(boost::bind(&PathPlan::goalCallback,this));
}

PathPlan::~PathPlan()
{
    delete _rate;
}

void PathPlan::start(const unsigned int rate)
{
    delete _rate;
    _loopRate = rate;
    _rate = new ros::Rate(_loopRate);
    //start action
    _actionMoveTo->start();
    this->run();
}

void PathPlan::run()
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

void PathPlan::goalCallback()
{
   ROS_INFO("Received goal");
   ohm_path_plan::MoveToGoalConstPtr goal = _actionMoveTo->acceptNewGoal();

   _pathStart = goal->start;
   _pathEnd   = goal->end;

   //send feedback
   _actionMoveTo_feedback.isPlaning = true;
   _actionMoveTo->publishFeedback(_actionMoveTo_feedback);

   //fill map
   char* data = (char*)&_map.data[0];
   double cellSize = _map.info.resolution;
   unsigned int width = _map.info.width;
   unsigned int height = _map.info.height;

   printf("map data: cellSize: %f, resol: %dx%d\n",cellSize,width,height);
   printf("          origin:  (%f,%f),\n", _map.info.origin.position.x, _map.info.origin.position.y);

   obvious::AStarMap* map = obvious::AStarMap::create(data, cellSize, width, height);

   ROS_INFO("inflate map ...");
   map->inflate(_robotRadius);

   //get path
   unsigned int xStart = (unsigned int)((_pathStart.pose.position.x - _map.info.origin.position.x) / cellSize + 0.555);
   unsigned int yStart = (unsigned int)((_pathStart.pose.position.y - _map.info.origin.position.y) / cellSize + 0.555);
   unsigned int xEnd   = (unsigned int)((_pathEnd.pose.position.x   - _map.info.origin.position.x) / cellSize + 0.555);
   unsigned int yEnd   = (unsigned int)((_pathEnd.pose.position.y   - _map.info.origin.position.y) / cellSize + 0.555);

   printf("start: (%d,%d), end: (%d,%d)\n", xStart, yStart, xEnd, yEnd);

   ROS_INFO("now planing path....");
   std::vector<unsigned int> path_raw = obvious::AStar::pathFind(map, xStart, yStart, xEnd, yEnd);
   ROS_INFO("Found %d wps",(int)path_raw.size());
   if(path_raw.size() == 0)
   {
      _actionMoveTo_result.succes = false;
      _actionMoveTo->setAborted(_actionMoveTo_result);
      return;
   }
   //convert path
   std::vector<obvious::AStarCoord> path = map->translatePathToCoords(path_raw, xStart, yStart);

   nav_msgs::Path msgPath;
   msgPath.header.frame_id = _frame_id;
   for (unsigned int i = 0; i < path.size(); ++i)
   {
      geometry_msgs::PoseStamped tmp;
      tmp.pose.position.x = path[i].x + _map.info.origin.position.x;
      tmp.pose.position.y = path[i].y + _map.info.origin.position.y;
      tmp.pose.position.z = 0;
      tmp.pose.orientation.w = 0;
      tmp.pose.orientation.x = 0;
      tmp.pose.orientation.y = 0;
      tmp.pose.orientation.z = 0;
      tmp.header.frame_id = _frame_id;

      msgPath.poses.push_back(tmp);
   }

   //set orientation of last wp
   msgPath.poses[msgPath.poses.size() - 1].pose.orientation = _pathEnd.pose.orientation;

   //send feedback
   _actionMoveTo_feedback.isMoving = false;
   _actionMoveTo_feedback.isPlaning = false;
   _actionMoveTo_feedback.currentWaypoint = 0;
   _actionMoveTo_feedback.numWaypoints = msgPath.poses.size();
   _actionMoveTo->publishFeedback(_actionMoveTo_feedback);

   //publsih path
   _pubPath.publish(msgPath);

   //todo
   //receive feedback from pathcontroller
   _actionMoveTo_result.succes = true;
   _actionMoveTo->setSucceeded(_actionMoveTo_result);
}

void PathPlan::subCallback_map(const nav_msgs::OccupancyGrid& msg)
{
   _map = msg;
}


//---------------------------------------------------------------------------------------------------------------------

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ohm_path_plan_node");
    ros::NodeHandle nh("~");

    PathPlan node;
    node.start(10);

}



