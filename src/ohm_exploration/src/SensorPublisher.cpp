#include "SensorPublisher.h"
#include "Sensor.h"

#include <visualization_msgs/Marker.h>
#include <tf/tf.h>

#include <cmath>
#include <ostream>

SensorPublisher::SensorPublisher(const Sensor& sensor, ros::NodeHandle& nh)
    : _sensor(sensor),
      _pubSensor(nh.advertise<visualization_msgs::Marker>("sensor_beam", 2))
{

}

void SensorPublisher::publish(const tf::Transform& pose)
{
    /* fill marker */
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time();
    marker.ns = "";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = pose.getOrigin()[0];
    marker.pose.position.y = pose.getOrigin()[1];
    marker.pose.position.z = pose.getOrigin()[2];

    tf::Quaternion orientation(pose.getRotation());
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    marker.scale.x = 1.0f;
    marker.scale.y = 1.0f;
    marker.scale.z = 1.0f;

    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;


    /* calc triangles */
    const float dH = std::tan(_sensor.beamAngleH() * 0.5f) * _sensor.range();
    const float dV = std::tan(_sensor.beamAngleV() * 0.5f) * _sensor.range();


    /* right triangle */
    geometry_msgs::Point point;

    point.x = 0.0f;
    point.y = 0.0f;
    point.z = 0.0f;
    marker.points.push_back(point);

    point.x = _sensor.range();
    point.y = -dH;
    point.z = dV;
    marker.points.push_back(point);

    point.z = -dV;
    marker.points.push_back(point);


    std_msgs::ColorRGBA color;

    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;
    color.a = 1.0f;
    marker.colors.push_back(color);

    color.a = 0.0f;
    marker.colors.push_back(color);
    marker.colors.push_back(color);


    /* left triangle */
    marker.points.push_back(marker.points[0]);

    point.x = _sensor.range();
    point.y = dH;
    point.z = dV;
    marker.points.push_back(point);

    point.z = -dV;
    marker.points.push_back(point);

    marker.colors.push_back(marker.colors[0]);
    marker.colors.push_back(marker.colors[1]);
    marker.colors.push_back(marker.colors[2]);


    /* bottom triangle */
    marker.points.push_back(marker.points[0]);
    marker.points.push_back(marker.points[2]);
    marker.points.push_back(marker.points[5]);
    marker.colors.push_back(marker.colors[0]);
    marker.colors.push_back(marker.colors[1]);
    marker.colors.push_back(marker.colors[2]);


    /* top triangle */
    marker.points.push_back(marker.points[0]);
    marker.points.push_back(marker.points[1]);
    marker.points.push_back(marker.points[4]);
    marker.colors.push_back(marker.colors[0]);
    marker.colors.push_back(marker.colors[1]);
    marker.colors.push_back(marker.colors[2]);


    _pubSensor.publish(marker);
}
