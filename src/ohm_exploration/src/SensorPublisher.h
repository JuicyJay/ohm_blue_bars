#ifndef ___SENSOR_PUBLISHER_H___
#define ___SENSOR_PUBLISHER_H___

#include <ros/ros.h>

class Sensor;

namespace tf {
class Transform;
}

class SensorPublisher
{
public:
    SensorPublisher(const Sensor& sensor, ros::NodeHandle& nh);

    void publish(const tf::Transform& pose);

private:
    const Sensor& _sensor;
    ros::Publisher _pubSensor;
};

#endif
