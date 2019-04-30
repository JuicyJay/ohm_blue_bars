/*
 * autonomy_switch.cpp
 *
 *  Created on: Apr 20, 2019
 *      Author: phil
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <iostream>
#include <string>

void callBackJoy(const sensor_msgs::Joy& joy);
void callBackVelAutonoy(const geometry_msgs::Twist& vel);
void callBackVelTeleop(const geometry_msgs::Twist& vel);
void callbackTimer(const ros::TimerEvent&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autonomy_switch");
  ros::NodeHandle nh;
  ros::NodeHandle prvNh("~");



  ros::Timer timer = nh.createTimer(ros::Duration(0.1), callbackTimer);
}

void callBackVelAutonoy(const geometry_msgs::Twist& vel)
{

}

void callBackVelTeleop(const geometry_msgs::Twist& vel)
{

}

void callBackJoy(const sensor_msgs::Joy& joy)
{

}
