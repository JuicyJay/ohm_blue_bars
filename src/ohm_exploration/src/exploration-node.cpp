#include <ros/ros.h>
#include <tf/tf.h>

#include "Sensor.h"
#include "SensorPublisher.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    Sensor sensor;
    SensorPublisher pubSensor(sensor, nh);
    ros::Rate rate(10);

    while (ros::ok())
    {
        tf::Transform transform;

        transform.setOrigin(tf::Vector3(0.0f, 0.0f, 1.0f));

        pubSensor.publish(transform);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
