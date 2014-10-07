#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "Sensor.h"
#include "SensorPublisher.h"

tf::StampedTransform _transform;

void callbackPose(const geometry_msgs::PoseStamped& pose)
{
    _transform.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    _transform.setRotation(tf::Quaternion(pose.pose.orientation.x,
                                          pose.pose.orientation.y,
                                          pose.pose.orientation.z,
                                          pose.pose.orientation.w));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "exploration");
    ros::NodeHandle nh;
    Sensor sensor;
    SensorPublisher pubSensor(sensor, nh);
    ros::Rate rate(10);
    ros::Subscriber subPose(nh.subscribe("slam_out_pose", 1, callbackPose));
//    tf::TransformListener listener;

    while (ros::ok())
    {

        pubSensor.publish(_transform);
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

    return 0;
}
