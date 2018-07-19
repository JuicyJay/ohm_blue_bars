/*
 * blue_bar_node.cpp
 *
 *  Created on: Jul 18, 2018
 *      Author: ninahetterich
 */

#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include "ohm_blue_bars/BlueBarsCfgConfig.h"



static image_transport::Publisher _pubWarped;
static image_transport::Publisher _pubColorFilter;

double alpha_ = 50.0;
double beta_ = 90.0;
double gamma_ = 90.0;
double f_ = 500.0;
double dist_ = 2000.0;

int Bmax = 255;
int Bmin = 100;
int Rmax = 200;
int Rmin = 0;
int Gmax = 100;
int Gmin = 0;


void callback(ohm_blue_bars::BlueBarsCfgConfig& config, uint32_t level)
{
  alpha_ = config.alpha;
  beta_  = config.beta;
  gamma_ = config.gamma;
  dist_  = config.dist;
 // std::cout << __PRETTY_FUNCTION__ << alpha_ << " " << beta_ << " " << gamma_ << " " << dist_ << std::endl;

  Bmax = config.Bmax;
  Bmin = config.Bmin;
  Rmax = config.Rmax;
  Rmin = config.Rmin;
  Gmax = config.Gmax;
  Gmin = config.Gmin;



}

void warpImage(const cv::Mat& input, cv::Mat& warped)
{
  double f = 0.0;
  double dist = 0.0;
  double alpha, beta, gamma;
  alpha = ((double) alpha_ - 90.) * M_PI / 180;
  beta = ((double) beta_ - 90.) * M_PI / 180;
  gamma = ((double) gamma_ - 90.) * M_PI / 180;
  f = (double) f_;
  dist = (double) dist_;
  cv::Size taille = input.size();
  double w = (double) taille.width, h = (double) taille.height;
  cv::Mat A1 =
      (cv::Mat_<float>(4, 3) << 1, 0, -w / 2, 0, 1, -h / 2, 0, 0, 0, 0, 0, 1);

  // Rotation matrices around the X,Y,Z axis
  cv::Mat RX =
      (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, std::cos(alpha), -std::sin(alpha), 0, 0, std::sin(
          alpha), std::cos(alpha), 0, 0, 0, 0, 1);

  cv::Mat RY =
      (cv::Mat_<float>(4, 4) << std::cos(beta), 0, -std::sin(beta), 0, 0, 1, 0, 0, std::sin(
          beta), 0, std::cos(beta), 0, 0, 0, 0, 1);

  cv::Mat RZ = (cv::Mat_<float>(4, 4) << std::cos(gamma), -std::sin(gamma), 0, 0, std::sin(
      gamma), std::cos(gamma), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

  // Composed Rotationsmatrix mit (RX,RY,RZ)
  cv::Mat R = RX * RY * RZ;

  // Translation matrix on the Z axis change dist will change the height
  cv::Mat T =
      (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, dist, 0, 0, 0, 1); // Camera Intrisecs matrix 3D -> 2D
  cv::Mat A2 =
      (cv::Mat_<float>(3, 4) << f, 0, w / 2, 0, 0, f, h / 2, 0, 0, 0, 1, 0);

  // Final and overall transformation matrix
  cv::Mat transfo = A2 * (T * (R * A1));

  // Apply matrix transformation
  warpPerspective(input, warped, transfo, taille,
      cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}

void ColorDetection(const cv::Mat& warped, cv::Mat& blueFilter)
{

  inRange(warped, cv::Scalar(Bmin,Rmin,Gmin), cv::Scalar(Bmax,Rmax,Gmax), blueFilter); // BRG


}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image;
  try
  {
    image = cv_bridge::toCvCopy(msg, "rgb8")->image;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    return;
  }
  cv::Mat warped;
  warpImage(image, warped);

  if(_pubWarped.getNumSubscribers())
  {
    cv_bridge::CvImage cvImage;
    cvImage.image = warped;
    cvImage.encoding = msg->encoding;
    _pubWarped.publish(cvImage.toImageMsg());
  }


  cv::Mat blueFilter = warped;
  ColorDetection(warped, blueFilter);

  if(_pubColorFilter.getNumSubscribers())
  {
    cv_bridge::CvImage cvImage;
    cvImage.image = blueFilter;

    cvImage.encoding = "mono8";//msg->encoding;
    sensor_msgs::ImagePtr imageRos = cvImage.toImageMsg();
    std::cout << "width " << blueFilter.cols << " height " << blueFilter.rows << std::endl;
//    imageRos->height = blueFilter.rows;
//    imageRos->width = blueFilter.cols;
   // imageRos->step = blueFilter.cols * 3;
    std::cout << " step = " << imageRos->step << std::endl;
    _pubColorFilter.publish(imageRos);
  }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  _pubWarped = it.advertise("warped", 1);
  _pubColorFilter = it.advertise("color_detected", 1);



  dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig> server;
  dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::spin();
  // cv::destroyWindow("view");
}

