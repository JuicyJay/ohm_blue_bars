
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
    _subState = _nh.subscribe("/path_control/state", 1, &PathPlan::subReached_callback, this);

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
   //measure Time:
   boost::posix_time::ptime start_t;
   boost::posix_time::ptime end_t;
   boost::posix_time::time_duration dur;
   start_t = boost::posix_time::microsec_clock::local_time();

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

   //debug
   unsigned char* buffer = new unsigned char[width * height * 3];
   map->convertToImage(buffer);
   obvious::serializePPM("/tmp/sim_map_raw.ppm",buffer, width, height, false);

   ROS_INFO("inflate map ...");
   map->inflate(_robotRadius);

   //debug
   map->convertToImage(buffer);
   obvious::serializePPM("/tmp/sim_map_inf.ppm",buffer, width, height, false);

   //get path
   obvious::Point2D start;
   obvious::Point2D target;
   start.x  = (_pathStart.pose.position.x - _map.info.origin.position.x) - _map.info.width * _map.info.resolution * 0.5;
   start.y  = (_pathStart.pose.position.y - _map.info.origin.position.y) - _map.info.height * _map.info.resolution * 0.5;
   target.x = (_pathEnd.pose.position.x   - _map.info.origin.position.x) - _map.info.width * _map.info.resolution * 0.5;
   target.y = (_pathEnd.pose.position.y   - _map.info.origin.position.y) - _map.info.height * _map.info.resolution * 0.5;

   printf("start: (%f,%f), end: (%f,%f)\n", start.x, start.y, target.x, target.y);

   ROS_INFO("now planing path....");
   std::vector<unsigned int> path_raw = obvious::AStar::pathFind(map, start, target);
   ROS_INFO("Found %d wps",(int)path_raw.size());
   if(path_raw.size() == 0)
   {
      end_t = boost::posix_time::microsec_clock::local_time();
      dur = start_t - end_t;
      ROS_INFO("Duration: %ds, %dms",(int) dur.total_milliseconds() / 1000, (int)dur.total_milliseconds() % 1000);
      _actionMoveTo_result.succes = false;
      _actionMoveTo->setAborted(_actionMoveTo_result);
      return;
   }
   //convert path
   std::vector<obvious::Point2D> path = map->translatePathToCoords(path_raw, start);

   //debug
   std::vector<unsigned int> mapIdx = map->translatePathToMapIndices(path_raw, start);

   for(std::vector<unsigned int>::iterator it=mapIdx.begin(); it!=mapIdx.end(); ++it)
   {
     buffer[3*(*it)] = 0;
     buffer[3*(*it)+1] = 0;
     buffer[3*(*it)+2] = 0;
   }

   obvious::serializePPM("/tmp/sim_path.ppm", buffer, width, height, false);
   delete[] buffer;


   nav_msgs::Path msgPath;
   msgPath.header.frame_id = _frame_id;
   for (unsigned int i = 0; i < path.size(); ++i)
   {
      geometry_msgs::PoseStamped tmp;
      tmp.pose.position.x = path[i].x + _map.info.origin.position.x + _map.info.width * _map.info.resolution * 0.5;
      tmp.pose.position.y = path[i].y + _map.info.origin.position.y + _map.info.height * _map.info.resolution * 0.5;
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


   end_t = boost::posix_time::microsec_clock::local_time();
   dur = start_t - end_t;
   ROS_INFO("Duration: %ds, %dms", (int)dur.total_milliseconds() / 1000,(int) dur.total_milliseconds() % 1000);
}

void PathPlan::subReached_callback(const std_msgs::Bool& msg)
{
   if(msg.data)
   {
      ROS_INFO("PathPlan -> Reched target");
      _actionMoveTo_result.succes = true;
      _actionMoveTo->setSucceeded(_actionMoveTo_result);
   }
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



