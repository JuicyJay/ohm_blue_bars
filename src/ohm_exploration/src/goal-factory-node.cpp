#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <string>

#include <Eigen/Geometry>

#include "FindWall.h"

FindWall _wallFinder;
std::vector<Wall> _walls;
ros::Publisher _pubWalls;
ros::Publisher _pubGoals;

void sendWallMarkers(void)
{
    visualization_msgs::MarkerArray markers;

    for (std::vector<Wall>::const_iterator wall(_walls.begin()); wall < _walls.end(); ++wall)
        markers.markers.push_back(wall->getMarkerMessage());

    _pubWalls.publish(markers);
}

void sendGoals(void)
{
    geometry_msgs::PoseArray goals;
    static unsigned int seq = 0;

    goals.header.seq = seq++;
    goals.header.stamp = ros::Time::now();
    goals.header.frame_id = "map";

    for (std::vector<Wall>::const_iterator wall(_walls.begin()); wall < _walls.end(); ++wall)
    {
        Eigen::Vector3f position(wall->center().x(), wall->center().y(), 0.0f);
        position *= wall->resolution();
        position += wall->origin();
        position.z() = 0.45f;

        Eigen::Vector2f n(wall->model().n());
        Eigen::Quaternionf orientation;

        if (wall->orientation() & Wall::Up || wall->orientation() & Wall::Right)
            n *= -1.0f;

        orientation.setFromTwoVectors(Eigen::Vector3f(1.0f, 0.0f, 0.0f), Eigen::Vector3f(n.x(), n.y(), 0.0f));
        orientation = orientation * Eigen::Quaternionf(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));


        geometry_msgs::Pose pose;

        pose.position.x = position.x();
        pose.position.y = position.y();
        pose.position.z = position.z();

        pose.orientation.x = orientation.x();
        pose.orientation.y = orientation.y();
        pose.orientation.z = orientation.z();
        pose.orientation.w = orientation.w();

        goals.poses.push_back(pose);
    }

    _pubGoals.publish(goals);
}

void callbackMap(const nav_msgs::OccupancyGrid& map)
{
    _walls.clear();
    _wallFinder.setMap(map);
    _wallFinder.search(_walls);

    sendWallMarkers();
    sendGoals();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "goal_factory");
    ros::NodeHandle para("~");
    ros::NodeHandle nh;
    std::string topic;

    para.param<std::string>("map_topic", topic, "/map");
    ros::Subscriber subMap(nh.subscribe(topic, 1, callbackMap));
    para.param<std::string>("marker_topic", topic, "/walls");
    _pubWalls = nh.advertise<visualization_msgs::MarkerArray>(topic, 2);
    para.param<std::string>("goal_topic", topic, "/goals");
    _pubGoals = nh.advertise<geometry_msgs::PoseArray>(topic, 2);

    ros::spin();
}
