/*
 * blue_bar_node.cpp
 *
 *  Created on: Jul 18, 2018
 *      Author: ninahetterich
 */

// Linie wieder reinlegen! (statt circle) läuft das? -läuft? -läuft!
// macht skeleton sinn? Ränder machen unfug wenn nur halbe Linie drin -> erstmal wieder rausgemacht
// Binärbild? -> sollte klappen, findet aber trotzdem noch Punkte wo nichts sein dürfte :(
// rechte Seite angepasst -> + 150 Spalten
// Fehler am unteren Bildrand ausgeblendet -> - 100 Reihen
//dynamic reconfigure? wie aktualisieren


#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <opencv-3.3.1-dev/opencv2/ximgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include "ohm_blue_bars/BlueBarsCfgConfig.h"
#include "Straight2D.h"
#include <Eigen/Dense>

static image_transport::Publisher _pubWarped;
static image_transport::Publisher _pubColorFilter;
static image_transport::Publisher _pubColorFilterDilated;
static image_transport::Publisher _pubCanny;
static image_transport::Publisher _pubThinned;
static image_transport::Publisher _pubLines;

//double alpha_ = 50.0;
//double beta_ = 90.0;
//double gamma_ = 90.0;
//double f_ = 500.0;
//double dist_ = 2000.0;  //zoom
//double dist;

int thresh_ = 0;
int threshScore_ = 60;
int distPoint_ = 4;  //max Abstand von Punkten um Teil der Linie zu sein
int distLine_ = 0;  //max Abstand zw Linien um als gleich zu gelten
int threshPxlDetect_ = 0;

//int Bmax = 255;
//int Bmin = 100;
//int Rmax = 200;
//int Rmin = 0;
//int Gmax = 100;
//int Gmin = 0;

int sizeA = 0;
int sizeB = 0;
int pointA = 0;
int pointB = 0;

struct Pixel
{
  Pixel(const unsigned int u, const unsigned int v) :
      u(u), v(v)
  {
  }
  unsigned int u;
  unsigned int v;
};

struct ScoredLine
{
  ScoredLine(Straight2D& line, const double score) :
      line(line), score(score)
  {
  }
  Straight2D line;
  double score;
};

namespace
{

const unsigned int RAN_TRIALS = 30;
}

void callback(ohm_blue_bars::BlueBarsCfgConfig& config, uint32_t level)
{
//  alpha_ = config.alpha;
//  beta_ = config.beta;
//  gamma_ = config.gamma;
//  dist_ = config.dist;
  // std::cout << __PRETTY_FUNCTION__ << alpha_ << " " << beta_ << " " << gamma_ << " " << dist_ << std::endl;

//  Bmax = config.Bmax;
//  Bmin = config.Bmin;
//  Rmax = config.Rmax;
//  Rmin = config.Rmin;
//  Gmax = config.Gmax;
//  Gmin = config.Gmin;

  sizeA = config.sizeA;
  sizeB = config.sizeB;
  pointA = config.pointA;
  pointB = config.pointB;
  //threshPxlDetect_ = config.thresh_pxl_detect;
  //threshScore_ = config.threshScore; // ->

}

void warpImage(const cv::Mat& input, cv::Mat& warped)
{
//  double f = 0.0;
//  double dist = 0.0;
//  double alpha, beta, gamma;
//  alpha = ((double)alpha_ - 90.) * M_PI / 180;
//  beta = ((double)beta_ - 90.) * M_PI / 180;
//  gamma = ((double)gamma_ - 90.) * M_PI / 180;
//  f = (double)f_;
//  dist = (double)dist_;
//  cv::Size taille = input.size();
//  double w = (double)taille.width, h = (double)taille.height;
//  cv::Mat A1 = (cv::Mat_<float>(4, 3) << 1, 0, -w / 2, 0, 1, -h / 2, 0, 0, 0, 0, 0, 1);
//
//  // Rotation matrices around the X,Y,Z axis
//  cv::Mat RX = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, std::cos(alpha), -std::sin(alpha), 0, 0, std::sin(alpha), std::cos(alpha), 0, 0, 0, 0, 1);
//
//  cv::Mat RY = (cv::Mat_<float>(4, 4) << std::cos(beta), 0, -std::sin(beta), 0, 0, 1, 0, 0, std::sin(beta), 0, std::cos(beta), 0, 0, 0, 0, 1);
//
//  cv::Mat RZ = (cv::Mat_<float>(4, 4) << std::cos(gamma), -std::sin(gamma), 0, 0, std::sin(gamma), std::cos(gamma), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
//
//  // Composed Rotationsmatrix mit (RX,RY,RZ)
//  cv::Mat R = RX * RY * RZ;
//
//  // Translation matrix on the Z axis change dist will change the height
//  cv::Mat T = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, dist, 0, 0, 0, 1);  // Camera Intrisecs matrix 3D -> 2D
//  cv::Mat A2 = (cv::Mat_<float>(3, 4) << f, 0, w / 2, 0, 0, f, h / 2, 0, 0, 0, 1, 0);
//
//  // Final and overall transformation matrix
//  cv::Mat transfo = A2 * (T * (R * A1));
//
//  // Apply matrix transformation
//  warpPerspective(input, warped, transfo, taille, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
//
//  //std::cout << "Fehler 1" << std::endl;
}

void ColorDetection(const cv::Mat& warped, cv::Mat& blueFilter)
{
//  cv::Mat blueFilter (blueFilter.size(), CV_8U); -> fail
//  cv::cvtColor(warped, warped, CV_BGR2HSV); -> fail
//  inRange(warped, cv::Scalar(Bmin, Rmin, Gmin), cv::Scalar(Bmax, Rmax, Gmax), blueFilter);  // BRG


  std::cout << "blueFilter Type" << blueFilter.type() << std::endl;



 // std::cout << "Fehler 2" << std::endl;

}

void CannyDetection(const cv::Mat& blueFilter, cv::Mat& edges)
{
  sizeA = 7;
  sizeB = 7;
  pointA = 3;
  pointB = 3;
  //std::cout << sizeA << " " << sizeB << " " << pointA << " " << pointB << std::endl;
  cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(sizeA, sizeB), cv::Point(pointA, pointB));
  //std::cout << "Reihen & Spalten" << element.rows << " " << element.cols << std::endl;
  dilate(blueFilter, blueFilter, element);
  erode(blueFilter, blueFilter, element);
  Canny(blueFilter, edges, 0, 100, 3);
  std::cout << "canny Type" << edges.type() << std::endl;
//  std::cout << " Fehler 3" << std::endl;

}

//void Thinned(const cv::Mat& blueFilter, cv::Mat& thinned)
//{
//
//
//  std::cout << " Fehler 3.5 " << std::endl;
//
//  cv::ximgproc::thinning(blueFilter, thinned, cv::ximgproc::THINNING_ZHANGSUEN);
//
//
//
////  unsigned int minHorizontal = 200;
////  unsigned int maxHorizontal = 300;
////
////  for (unsigned int i = 0; i < thinned.rows - 3; i++) {
////         if ((i < minHorizontal) || (i > maxHorizontal)) {
////             for (unsigned int j = 0; j < thinned.cols - 1; j++) {
////                 thinned.at<int>(i, j) = 0;
////             }
////         }
////     }
////
////
//
//  std::cout << "Fehler 4" << std::endl;
//
//}


void LineDetection(const cv::Mat& edges, cv::Mat& destination)

{



  //Binary image
  destination = edges;
  cv::Mat binaryMat(edges.size(), edges.type());

  //std::cout << "edges size & type" << edges.size() << edges.type() << std::endl;


  //std::cout << "Fehler 4.1" << std::endl;



  //Apply thresholding
  cv::threshold(edges, binaryMat, thresh_, 255, 0); //cv::THRESH_BINARY); warum binaryMat??


  std::vector<Pixel> pixels;

  for(unsigned int i = 0; i < binaryMat.rows +150 ; i++) //warum binaryMat??
    for(unsigned int j = 0; j < binaryMat.cols -100 ; j++) //warum binaryMat??
    {
      //std::cout << pixels.size() << std::endl;

      if(static_cast<int>(binaryMat.at<uchar>(j, i)) > threshPxlDetect_) //>100 //warum binaryMat??
      {
        //          std::cout << " r c " << j << " " << i << std::endl;
        //          std::cout << static_cast<int>(binaryMat.at<uchar>(j,i)) << std::endl;
        pixels.push_back(Pixel(i, j));
        //     line(binaryMat, Point(300, 300), Point(i, j), Scalar(25, 225, 225), 2, CV_AA);
      }
//      else
//        std::cout << "binary " << static_cast<int>(binaryMat.at<uchar>(j, i)) << std::endl;

    }
  //    std::cout << " total pixels: " <<pixels.size() << std::endl;
  //    std::cout << " Position " <<pixels.size() << std::endl;
 // std::cout << "Fehler 4.2" << std::endl;

  Eigen::Vector3d vec;
  Straight2D xaxis(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(static_cast<double>(binaryMat.cols), 0.0)); //warum binaryMat??
  Straight2D xEdge(Eigen::Vector2d(0.0, static_cast<double>(binaryMat.rows)), Eigen::Vector2d(static_cast<double>(binaryMat.cols), static_cast<double>(binaryMat.rows))); //warum binaryMat??
//  std::cout << "Fehler 4.22" << std::endl;

  unsigned int trials = 0;
  std::vector<ScoredLine> linesFound;

  //std::cout << "Fehler 4.221" << std::endl;
  while(trials < RAN_TRIALS)
  {
   // std::cout << "Fehler 4.23" << std::endl;
    unsigned int idx1 = std::rand() % pixels.size();
    unsigned int idx2 = std::rand() % pixels.size();
  //  std::cout << "Fehler 4.24" << std::endl;

    if(idx1 == idx2)
      continue;

    Eigen::Vector2d point1(static_cast<double>(pixels[idx1].u), static_cast<double>(pixels[idx1].v));
    Eigen::Vector2d point2(static_cast<double>(pixels[idx2].u), static_cast<double>(pixels[idx2].v));
//
//    cv::Point cvP1(point1(1), point1(0));
//    cv::Point cvP2(point2(1), point2(0));
//    cv::line(destination, cvP1, cvP2, cv::Scalar(255, 100, 100), 2, CV_AA);
//    cv::circle(destination, cvP1, 15.0, cv::Scalar( 255, 255, 255 ), 1, 8 );
//    cv::circle(destination, cvP2, 15.0, cv::Scalar( 255, 255, 255 ), 1, 8 );

    Straight2D lineCandidate(point1, point2);
  //  std::cout << "Fehler 4.3" << std::endl;

    //generate score, zählt pixel mit mindest Abstand von 0.1
    double score = 0.0;
    for(unsigned int i = 0; i < pixels.size(); i++)
    {
      Eigen::Vector2d point(static_cast<double>(pixels[i].u), static_cast<double>(pixels[i].v));

      if(lineCandidate.distPointLine(point) < 0.1)                   //(static_cast<double>(distPoint_) * 0.1)) //-> may
        score += 1.0;  //-> may

    }
    if(score > static_cast<double>(100))//threshScore_
    {
      bool found = false;


  //      calculate dist between two lines

      for(unsigned int i = 0; i < linesFound.size(); i++)
      {

        Eigen::Vector2d cutLinesI0 = linesFound[i].line.cut(xaxis); //?
        Eigen::Vector2d cutLinesI1 = linesFound[i].line.cut(xEdge); //?

        Eigen::Vector2d cutCandidate0 = lineCandidate.cut(xaxis);
        Eigen::Vector2d cutCandidate1 = lineCandidate.cut(xEdge);

        Eigen::Vector2d differenz_xaxis = cutLinesI0 - cutCandidate0;
        double dist_xaxis = differenz_xaxis.norm();

        Eigen::Vector2d differenz_xEdge = cutLinesI1 - cutCandidate1;
        double dist_xEdge = differenz_xEdge.norm();

        if(dist_xaxis < (static_cast<double>(distLine_)))
        {
          //std::cout << "Dist = " << dist << std::endl;
          found = true;
          break;
        }

      }
      if(!found)
      linesFound.push_back(ScoredLine(lineCandidate, score));
   }


    trials++;


  //std::cout << "Fehler 5" << std::endl;

  }


  //std::cout << "Total lines found " << linesFound.size() << std::endl;

  double bestScore = 0.0;  // std::numeric_limits<double>::max();  -> may
  unsigned int bestIdx0 = std::numeric_limits<unsigned int>::max();
  unsigned int bestIdx1 = 0;

  for(unsigned int i = 0; i < linesFound.size(); i++)
  {
    // cv::Mat destination;

    Eigen::Vector2d cut10 = linesFound[i].line.cut(xaxis);
    Eigen::Vector2d cut11 = linesFound[i].line.cut(xEdge);

    std::cout << "test1" << std::endl;

        if((0 >= cut11(1) >= 480) && (0 >= cut10(1) >= 480) && (0 >= cut10(0) >= 640) && (0 >= cut11(0) >= 640))
                { return;
                }

      std::cout << "test2" << std::endl;


    cv::Point cvP11(cut10(0), cut10(1));
    cv::Point cvP12(cut11(0), cut11(1));

    cv::line(destination, cvP11, cvP12, cv::Scalar(255, 100, 100), 5, CV_AA);

    std::cout << "Punkt1:" << cvP11 <<"\n" << "Punkt2:"<< cvP12 << std::endl;



//    cv::circle(destination, cvP11, 32.0, cv::Scalar( 255, 255, 255 ), 1, 8 );
//    cv::circle(destination, cvP12, 32.0, cv::Scalar( 255, 255, 255 ), 1, 8 );

  //  for(unsigned int i = 0 < linesFound)
  }
  for(unsigned int i = 0; i < pixels.size(); i++)
  {

//    cv::Point cvP1(pixels[i].u, pixels[i].v);
//    cv::Point cvP2(point2(1), point2(0));
//    cv::line(destination, cvP1, cvP2, cv::Scalar(255, 100, 100), 2, CV_AA);
//    cv::circle(destination, cvP1, 15.0, cv::Scalar( 255, 255, 255 ), 1, 4 );
//    cv::circle(destination, cvP2, 15.0, cv::Scalar( 255, 255, 255 ), 1, 8 );
  }

//  std::cout << "Fehler 6" << std::endl;


}
//

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat image;
  try
  {
    image = cv_bridge::toCvCopy(msg, "rgb8")->image;
  }
  catch(cv_bridge::Exception& e)
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
  cv::Mat thinned;
  ColorDetection(warped, blueFilter);


  if(_pubColorFilter.getNumSubscribers())
  {
    cv_bridge::CvImage cvImage;
    cvImage.image = blueFilter;

    cvImage.encoding = "mono8";  //msg->encoding;
    sensor_msgs::ImagePtr imageRos = cvImage.toImageMsg();
 //   std::cout << "width " << blueFilter.cols << " height " << blueFilter.rows << std::endl;
//    imageRos->height = blueFilter.rows;
//    imageRos->width = blueFilter.cols;
    // imageRos->step = blueFilter.cols * 3;
 //   std::cout << " step = " << imageRos->step << std::endl;
    _pubColorFilter.publish(imageRos);
  }

  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(3, 3));
  cv::dilate(blueFilter, blueFilter, element);
  cv::erode(blueFilter, blueFilter, element);

  if(_pubColorFilterDilated.getNumSubscribers())
  {
    cv_bridge::CvImage cvImage;
    cvImage.image = blueFilter;

    cvImage.encoding = "mono8";  //msg->encoding;
    sensor_msgs::ImagePtr imageRos = cvImage.toImageMsg();
    std::cout << "width " << blueFilter.cols << " height " << blueFilter.rows << std::endl;
//    imageRos->height = blueFilter.rows;
//    imageRos->width = blueFilter.cols;
//     imageRos->step = blueFilter.cols * 3;
//    std::cout << " step = " << imageRos->step << std::endl;
    _pubColorFilterDilated.publish(imageRos);
  }

  cv::Mat edges;
  CannyDetection(blueFilter, edges);
  if(_pubCanny.getNumSubscribers())
  {

    cv_bridge::CvImage cvImageEdges;
    cvImageEdges.encoding = "mono8";
    cvImageEdges.image = edges;
    sensor_msgs::ImagePtr imageRosEdges = cvImageEdges.toImageMsg();
    _pubCanny.publish(imageRosEdges);
  }


//  Thinned(blueFilter, thinned);
//  if(_pubThinned.getNumSubscribers())
//  {
////    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7), cv::Point(3, 3));
////    cv::dilate(blueFilter, blueFilter, element);
////    cv::erode(blueFilter, blueFilter, element);
//
//    cv_bridge::CvImage cvImageThin;
//    cvImageThin.encoding = "mono8";
//    cvImageThin.image = thinned;
//    sensor_msgs::ImagePtr imageRosThinned = cvImageThin.toImageMsg();
//    _pubThinned.publish(imageRosThinned);
//  }


  cv::Mat destination;
  LineDetection(edges, destination);
  if(_pubLines.getNumSubscribers())
  {
    cv_bridge::CvImage cvImageLines;
    cvImageLines.encoding = "mono8";
    cvImageLines.image = destination;
    sensor_msgs::ImagePtr imageRosLines = cvImageLines.toImageMsg();
    _pubLines.publish(imageRosLines);
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "blue_bar_node");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  _pubWarped = it.advertise("warped", 1);
  _pubColorFilter = it.advertise("color_detected", 1);
  _pubColorFilterDilated = it.advertise("color_dilated", 1);
  _pubCanny = it.advertise("canny_filter", 1);
//  _pubThinned = it.advertise("thinned_image", 1);
  _pubLines = it.advertise("lines", 1);

  dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig> server;
  dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);

  ros::spin();
  // cv::destroyWindow("view");
}

