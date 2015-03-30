
#ifndef TEMPLATE_H_
#define TEMPLATE_H_

#define USE_OPENCV

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <obcore/base/Timer.h>

#include "PathFind/Map/GridMap.h"
#include "PathFind/AStar_dt/Astar_dt.h"
#include "PathFind/Map/MapOperations/MapOperations.h"

#include <ohm_path_plan/PlanPaths.h>
#include <ohm_path_plan/PlanPath.h>

class PathPlan_AStar
{
private:    //dataelements
    unsigned int _loopRate;

    ros::Rate* _rate;
    ros::NodeHandle _nh;

    ros::Publisher _pubPath;
    ros::Subscriber _subMap;
    ros::Subscriber _subTargetPose;

    ros::ServiceServer _srv_plan_paths;
    ros::ServiceServer _srv_plan_path;

    ros::ServiceServer _servicePlan;

    tf::TransformListener _tf_listnener;

    nav_msgs::OccupancyGrid _map;

    bool _gotMap;

    double _robot_radius;
    double _dt_radius;

    double _cost_short_step;
    double _cost_long_step ;
    double _factor_dist    ;
    double _costmap_weight ;

    std::string _frame_id;

    std::string _tf_map_frame;
    std::string _tf_robot_frame;

    //std::vector<apps::GridMap*> _cost_maps;

public:
    PathPlan_AStar();
    virtual ~PathPlan_AStar();

    /**
     * @fn void start(const unsigned int frames = 10)
     *
     * @brief
     *
     *
     * @param[in] const unsigned int rate  ->  rate of the working loop in [1/s]
     *
     *
     * @return  void
     */
    void start(const unsigned int rate = 10);

private:    //functions

    /**
     * @fn void run()
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();

    void do_map_operations(apps::Astar_dt* planner);
    std::vector<apps::Point2D> do_path_planning(apps::Astar_dt* planner, apps::Point2D start, apps::Point2D end);

    void debug_save_as_img(std::string file, apps::GridMap* map, std::vector<apps::Point2D>& path);



    nav_msgs::Path toRosPath(std::vector<apps::Point2D> path, geometry_msgs::PoseStamped target);

    void subCallback_map(const nav_msgs::OccupancyGrid& msg);
    void subCallback_target(const geometry_msgs::PoseStamped& msg);

    bool srvCallback_plan_sorted(ohm_path_plan::PlanPathsRequest& req,
                                 ohm_path_plan::PlanPathsResponse& res);
    bool srvCallback_plan_path(ohm_path_plan::PlanPathRequest& req,
                               ohm_path_plan::PlanPathResponse& res);

};

#endif /* TEMPLATE_H_ */
