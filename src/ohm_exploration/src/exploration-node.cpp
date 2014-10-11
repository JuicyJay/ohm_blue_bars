#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <ostream>

#include "Sensor.h"
#include "SensorPublisher.h"
#include "Grid.h"

tf::StampedTransform _transform;
nav_msgs::OccupancyGrid _map;
Grid* _grid = 0;

void callbackPose(const geometry_msgs::PoseStamped& pose)
{
    _transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    _transform.setRotation(tf::Quaternion(pose.pose.orientation.x,
                                          pose.pose.orientation.y,
                                          pose.pose.orientation.z,
                                          pose.pose.orientation.w));
}

void callbackMap(const nav_msgs::OccupancyGrid& map)
{
    _map = map;

    if (!_grid)
        _grid = new Grid(map, 8);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    Sensor sensor;
    SensorPublisher pubSensor(sensor, nh);
    ros::Rate rate(10);
    ros::Subscriber subPose(nh.subscribe("pose", 1, callbackPose));
    ros::Subscriber subMap(nh.subscribe("map", 1, callbackMap));

//    tf::TransformListener listener;
    _transform.setRotation(tf::Quaternion(0.0f, 0.0f, M_PI * 0.25f));

    ros::Publisher pubGrid(nh.advertise<nav_msgs::GridCells>("grid", 2));
    ros::Publisher pubWall(nh.advertise<nav_msgs::GridCells>("walls", 2));

    while (ros::ok())
    {
//        std::cout << "transform: " << _transform.getRotation() << std::endl;
        pubSensor.publish(_transform);

        if (_grid)
        {
            pubGrid.publish(_grid->getGridCellMessage());
            pubWall.publish(_grid->getWallGridCells());
            _grid->update(_map, sensor, _transform);
        }

//        if (listener.waitForTransform("laser", "map", ros::Time::now(), ros::Duration(0.1f)))
//        try
//        {
//            listener.lookupTransform("laser", "map", ros::Time(0), transform);
//            pubSensor.publish(transform);
//        }
//        catch (tf::TransformException ex)
//        {
//            ROS_ERROR("%s", ex.what());
//        }

        ros::spinOnce();
        rate.sleep();
    }

    delete _grid;

    return 0;
}
