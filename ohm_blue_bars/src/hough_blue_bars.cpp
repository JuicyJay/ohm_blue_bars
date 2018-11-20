/*
 * hough_blue_bars.cpp
 *
 *  Created on: Sep 12, 2018
 *      Author: ninahetterich
 */

/*
 * dynamic reconfigure läuft
 * Schnittpunkte mit X-Achse laufen
 * Mittellinie läuft
 */

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
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>


//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/point_types.h>


static image_transport::Publisher _pubWarped;
static image_transport::Publisher _pubColorDetection;
static image_transport::Publisher _pubMorphOperations;
static image_transport::Publisher _pubSkeleton;
static image_transport::Publisher _pubHough;
static image_transport::Publisher _pubCenter;
static ros::Publisher _pubPath;
static pcl::PointCloud<pcl::PointXYZ> _cloud;

double f_ = 500.0;
double dist_ = 2000.0;
double alpha_ = 50.0;
double beta_ = 90.0;
double gamma_ = 90.0;

double Xcenter_0;
double Xcenter_1;

int Bmax = 255;
int Bmin = 100;
int Rmax = 200;
int Rmin = 0;
int Gmax = 100;
int Gmin = 0;

int intersections = 500;

int sizeA = 0;
int sizeB = 0;
int pointA = 0;
int pointB = 0;

void localizePixel(const cv::Point& pixel, geometry_msgs::Point& point);

void callback(ohm_blue_bars::BlueBarsCfgConfig& config, uint32_t level)
{

	std::cout << __PRETTY_FUNCTION__ << std::endl;

	alpha_ = config.alpha;
	beta_ = config.beta;
	gamma_ = config.gamma;
	dist_ = config.dist;

	Bmax = config.Bmax;
	Bmin = config.Bmin;
	Rmax = config.Rmax;
	Rmin = config.Rmin;
	Gmax = config.Gmax;
	Gmin = config.Gmin;

	intersections = config.intersections;

	sizeA = config.sizeA;
	sizeB = config.sizeB;
	pointA = config.pointA;
	pointB = config.pointB;

}

void warpimage(const cv::Mat& input, cv::Mat& warped)
{
	double f = 0.0;
	double dist = 0.0;
	double alpha;
	double beta;
	double gamma;

	alpha = ((double)alpha_ - 90.) * M_PI / 180;
	beta = ((double)beta_ - 90.) * M_PI / 180;
	gamma = ((double)gamma_ - 90.) * M_PI / 180;
	f = (double)f_;
	dist = (double)dist_;
	cv::Size taille = input.size();
	double w = (double)taille.width, h = (double)taille.height;
	cv::Mat A1 = (cv::Mat_<float>(4, 3) << 1, 0, -w / 2, 0, 1, -h / 2, 0, 0, 0, 0, 0, 1);

	// Rotation matrices around the X,Y,Z axis
	cv::Mat RX = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, std::cos(alpha), -std::sin(alpha), 0, 0, std::sin(alpha), std::cos(alpha), 0, 0, 0, 0, 1);

	cv::Mat RY = (cv::Mat_<float>(4, 4) << std::cos(beta), 0, -std::sin(beta), 0, 0, 1, 0, 0, std::sin(beta), 0, std::cos(beta), 0, 0, 0, 0, 1);

	cv::Mat RZ = (cv::Mat_<float>(4, 4) << std::cos(gamma), -std::sin(gamma), 0, 0, std::sin(gamma), std::cos(gamma), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

	//  Rotationsmatrix mit (RX,RY,RZ)
	cv::Mat R = RX * RY * RZ;

	// Translation matrix on the Z axis change dist will change the height
	cv::Mat T = (cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, dist, 0, 0, 0, 1);  // Camera Intrisecs matrix 3D -> 2D
	cv::Mat A2 = (cv::Mat_<float>(3, 4) << f, 0, w / 2, 0, 0, f, h / 2, 0, 0, 0, 1, 0);

	// Final and overall transformation matrix
	cv::Mat transfo = A2 * (T * (R * A1));

	// Apply matrix transformation
	warpPerspective(input, warped, transfo, taille, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

	//std::cout << "Fehler 1" << std::endl;
}

void colordetection(const cv::Mat& warped, cv::Mat& blueFilter)
{

	inRange(warped, cv::Scalar(Bmin, Rmin, Gmin), cv::Scalar(Bmax, Rmax, Gmax), blueFilter);  // BRG

}

void morphoperations(const cv::Mat& blueFilter)
{
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(sizeA, sizeB), cv::Point(pointA, pointB));
	dilate(blueFilter, blueFilter, element);
	erode(blueFilter, blueFilter, element);
	dilate(blueFilter, blueFilter, element);
	erode(blueFilter, blueFilter, element);
	dilate(blueFilter, blueFilter, element);
	erode(blueFilter, blueFilter, element);
	dilate(blueFilter, blueFilter, element);
	erode(blueFilter, blueFilter, element);
	dilate(blueFilter, blueFilter, element);
	erode(blueFilter, blueFilter, element);
	dilate(blueFilter, blueFilter, element);
	erode(blueFilter, blueFilter, element);
}

void skeleton(const cv::Mat& blueFilter, cv::Mat& thinned)
{
	cv::ximgproc::thinning(blueFilter, thinned, cv::ximgproc::THINNING_ZHANGSUEN);

	unsigned int minHorizontal = 40;
	unsigned int maxHorizontal = 460;
	for(unsigned int i = 0; i < thinned.rows - 3; i++)
	{
		if((i < minHorizontal) || (i > maxHorizontal))
		{
			for(unsigned int j = 0; j < thinned.cols - 1; j++)
			{
				thinned.at<int>(i, j) = 0;
			}
		}
	}
}

void houghdetection(const cv::Mat& thinned, cv::Mat& warped, std::vector<cv::Vec2f>& lines)
{

	//std::cout << "pt 1 = (lalala) " << pt1 << std::endl;
	//std::cout << "Hough" << std::endl;

	//std::vector<cv::Vec2f> lines;
	HoughLines(thinned, lines, 1, CV_PI / 180, intersections, 0, 0);
	std::cout << __PRETTY_FUNCTION__ << " lines deteced " << lines.size() << std::endl;
	//std::cout << cv::Pon

	for(size_t i = 0; i < lines.size(); i++)
	{

		//cv::Mat warped;
		float rho = lines[i][0];
		float theta = lines[i][1];
		cv::Point pt1;
		cv::Point pt2;
		double a = cos(theta);
		double b = sin(theta);
		double x0 = a * rho;
		double y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(warped, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);

		//  std::cout << "pt1" << pt1 << "\n" <<"pt2" << pt2 << std::endl;
		std::cout << "What's in lines: " << lines[i] << std::endl;
		//  std::cout << "a:" << a << "\n" << "b:" << b << "\n" << "x0:" << x0 << "\n" << "y0:" << y0 << std::endl;
	}
	// std::cout << "pt 1 = (lalalaLUEEEE) " << pt1 << std::endl;
	//   std::cout << "Warped-type:" << warped.type() << std::endl;

}

void centerline(const std::vector<cv::Vec2f>& lines, cv::Mat& center, cv::Mat& warped, cv::Point& centerpt_0, cv::Point& centerpt_1)
{
	if(!lines.size())
	{
		std::cout << __PRETTY_FUNCTION__ << " error! Found no line " << std::endl;
		return;
	}
	if(lines.size() != 2)
	{
		std::cout << __PRETTY_FUNCTION__ << " error! Found invalid size of lines " << lines.size() << " should be 2 "
				<< std::endl;
		return;
	}

	// schnittpunkt der Linien mit X-Achsen

	std::vector<Eigen::Vector2d> cutLineVector0;
	std::vector<Eigen::Vector2d> cutLineVector1;

	for(size_t i = 0; i < lines.size(); i++)
	{

		//cv::Mat warped;
		float rho = lines[i][0];
		float theta = lines[i][1];
		cv::Point pt1;
		cv::Point pt2;

		double a = cos(theta);
		double b = sin(theta);
		double x0 = a * rho;
		double y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));

		//   std::cout << "pt1.x: " << pt1.x << "\npt1.y: " << pt1.y << std::endl;
		//   std::cout << "pt2.x: " << pt2.x << "\npt2.y: " << pt2.y << std::endl;

		Eigen::Vector2d pt1Vec(pt1.x, pt1.y);
		Eigen::Vector2d pt2Vec(pt2.x, pt2.y);

		Straight2D linestoCut(pt1Vec, pt2Vec);

		//   std::cout << "pt1Vec: " << pt1Vec << std::endl;

		cv::Mat binaryMat(warped.size(), warped.type());
		// Eigen::Vector3d vec;
		Straight2D xaxis(Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(static_cast<double>(binaryMat.cols), 0.0));
		Straight2D xEdge(Eigen::Vector2d(0.0, static_cast<double>(binaryMat.rows)), Eigen::Vector2d(static_cast<double>(binaryMat.cols), static_cast<double>(binaryMat.rows)));

		Eigen::Vector2d cutLine0 = linestoCut.cut(xaxis);  //oberer Schnittpunkt mit X-Achse als Vector
		Eigen::Vector2d cutLine1 = linestoCut.cut(xEdge);  //unterer Schnittpunkt mit X-Achse als Vector
		cutLineVector0.push_back(cutLine0);
		cutLineVector1.push_back(cutLine1);
		std::cout << "Schnittpunkt mit x-Achse oben: \n" << cutLine0[0] << " & " << cutLine0[1] << std::endl;
		//std::cout << "Schnittpunkt mit x-Achse unten: \n" << cutLine1 << std::endl;

		center = warped;

		cv::Point var1(cutLine0(0), cutLine0(1));  //Schnittpunkt oben als Point
		cv::Point var2(cutLine1(0), cutLine1(1));  //Schnittpunkt unten als Point


		cv::circle(center, var1, 20.0, cv::Scalar(173, 255, 47), 3, 8);  //grüner kreis
		cv::circle(center, var2, 20.0, cv::Scalar(255, 0, 0), 3, 8);  //roter kreis

	}

	double middle0 = 0.0;
	for(unsigned int i = 0; i < cutLineVector0.size(); i++)
	{
		middle0 += cutLineVector0[i].x();
	}
	middle0 /= static_cast<double>(cutLineVector0.size());
	std::cout << __PRETTY_FUNCTION__ << " middle0 = " << middle0 << std::endl;
	double xaxis_min = 0.0;
	cv::Point abs_centerPoint0(middle0, xaxis_min);
	cv::circle(center, abs_centerPoint0, 20.0, cv::Scalar(100, 100, 255), 3, 8);  //blauer kreis
	centerpt_0 = abs_centerPoint0;


	double middle1 = 0.0;
	for(unsigned int i = 0; i < cutLineVector1.size(); i++)
	{
		middle1 += cutLineVector1[i].x();
	}
	middle1 /= static_cast<double>(cutLineVector1.size());
	std::cout << __PRETTY_FUNCTION__ << " middle1 = " << middle1 << std::endl;
	double xaxis_max = 480.0;
	cv::Point abs_centerPoint1(middle1, xaxis_max);
	cv::circle(center, abs_centerPoint1, 20.0, cv::Scalar(200, 100, 255), 3, 8);  //rosa kreis
	centerpt_1 = abs_centerPoint1;

	line(center, abs_centerPoint0, abs_centerPoint1, cv::Scalar(255, 255, 0), 3, CV_AA);

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	//std::cout << "image Callback" << std::endl;
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
	warpimage(image, warped);

	if(_pubWarped.getNumSubscribers())
	{
		cv_bridge::CvImage cvImage;
		cvImage.image = warped;
		cvImage.encoding = msg->encoding;
		_pubWarped.publish(cvImage.toImageMsg());
	}

	//std::cout << "image Callback warped" << std::endl;

	cv::Mat blueFilter = warped;  //warum = warped?
	colordetection(warped, blueFilter);

	if(_pubColorDetection.getNumSubscribers())
	{
		cv_bridge::CvImage cvImageColor;
		cvImageColor.image = blueFilter;

		cvImageColor.encoding = "mono8";  //msg->encoding;
		sensor_msgs::ImagePtr imageRosColor = cvImageColor.toImageMsg();
		//   std::cout << "width " << blueFilter.cols << " height " << blueFilter.rows << std::endl;
		//    imageRos->height = blueFilter.rows;
		//    imageRos->width = blueFilter.cols;
		// imageRos->step = blueFilter.cols * 3;
		//   std::cout << " step = " << imageRos->step << std::endl;
		_pubColorDetection.publish(imageRosColor);
	}

	morphoperations(blueFilter);
	if(_pubMorphOperations.getNumSubscribers())
	{
		cv_bridge::CvImage cvImageMorph;
		cvImageMorph.image = blueFilter;
		cvImageMorph.encoding = "mono8";  //msg->encoding;
		sensor_msgs::ImagePtr imageRosMorph = cvImageMorph.toImageMsg();
		_pubMorphOperations.publish(imageRosMorph);
	}

	cv::Mat thinned;
	skeleton(blueFilter, thinned);
	if(_pubSkeleton.getNumSubscribers())
	{
		cv_bridge::CvImage cvImageThinned;
		cvImageThinned.image = thinned;
		cvImageThinned.encoding = "mono8";
		sensor_msgs::ImagePtr imageRosThinned = cvImageThinned.toImageMsg();
		_pubSkeleton.publish(imageRosThinned);
	}

	//    cv::Point pt1;
	//    cv::Point pt2;
	std::vector<cv::Vec2f> lines;
	houghdetection(thinned, warped, lines);
	std::cout << __PRETTY_FUNCTION__ << " found " << lines.size() << " lines " << std::endl;
	if(_pubHough.getNumSubscribers())
	{
		cv_bridge::CvImage cvImageHough;
		cvImageHough.image = warped;
		cvImageHough.encoding = msg->encoding;
		sensor_msgs::ImagePtr imageRosHough = cvImageHough.toImageMsg();
		_pubHough.publish(imageRosHough);
	}

	cv::Mat center;
	//std::cout << "pt 1 in callback " << pt1 << std::endl;
	cv::Point centerpt_0;
	cv::Point centerpt_1;

	centerline(lines, center, warped, centerpt_0, centerpt_1); //centerpt_0/_1 notwendig?
	if(_pubCenter.getNumSubscribers())
	{
		cv_bridge::CvImage cvImageCenter;
		cvImageCenter.image = center;
		cvImageCenter.encoding = msg->encoding;
		sensor_msgs::ImagePtr imageRosCenter = cvImageCenter.toImageMsg();
		_pubCenter.publish(imageRosCenter);
	}


	geometry_msgs::Point point;
	geometry_msgs::Point point2;
	localizePixel(centerpt_0, point);
	localizePixel(centerpt_1, point2);
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pose2;
	pose.header.frame_id = "base_link";
	pose2.header.frame_id = "base_link";
	pose.pose.position = point;
	pose2.pose.position = point2;
	nav_msgs::Path path;
	path.header.frame_id = "base_link";
	//	geometry_msgs::PoseStamped dummy;
	//	for(unsigned int i = 0; i < 10; i++)
	//	{
	//		dummy.pose.position.x += 0.2;
	//		dummy.pose.position.y += 0.02;
	//		path.poses.push_back(dummy);
	//	}
	path.poses.push_back(pose);
	path.poses.push_back(pose2);
	_pubPath.publish(path);
}

void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
	std::cout << __PRETTY_FUNCTION__ << " pcl w h " << cloud.width << " " << cloud.height << std::endl;
	_cloud = cloud;
}

void localizePixel(const cv::Point& pixel, geometry_msgs::Point& point)
{
	static tf::TransformListener listener;
	std::string targetFrame = "base_link";
	tf::StampedTransform tf;
	if(!listener.waitForTransform(targetFrame, _cloud.header.frame_id, ros::Time(0), ros::Duration(3.0)))
	{
		std::cout << __PRETTY_FUNCTION__ << " timeout waiting for transform " << std::endl;
		return;
	}
	try
	{
		listener.lookupTransform(targetFrame, _cloud.header.frame_id, ros::Time(0), tf);
	}
	catch(tf::TransformException& ex)
	{
		ROS_ERROR("%s",ex.what());
		return;
	}
	std::cout << __PRETTY_FUNCTION__ << " pixel " << pixel.x << " " << pixel.y << std::endl;
	const unsigned int idx = pixel.x * _cloud.width + pixel.y;
	if(idx >= _cloud.size())
	{
		std::cout << __PRETTY_FUNCTION__ << " idx " << idx << " out of range " << _cloud.size() << std::endl;
		return;
	}
	pcl::PointXYZ pointPCL = _cloud.points[idx];
	tf::Vector3 vec(pointPCL.x, pointPCL.y, pointPCL.z);
	std::cout << __PRETTY_FUNCTION__ << " vecPreTf " << vec.x() << " " << vec.y() << " " << vec.z() << std::endl;
	tf::Vector3 vecTransformed = tf * vec;
	point.x = vec.x();
	point.y = vec.y();
	point.z = vec.z();
	std::cout << __PRETTY_FUNCTION__ << " pixel " << pixel << " point " << vecTransformed.x() << " " << vecTransformed.y() << " " << vecTransformed.z() << std::endl;
}
////test:
//void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud,const int u, const int v, cv::Point centerpt_1, cv::Point centerpt_2, geometry_msgs::Point& p)
//{
//
//
//  u = centerpt_1(0);
//  v = centerpt_1(1);
//
//
//  // get width and height of 2D point cloud data
//  int width = pCloud.width;
//  int height = pCloud.height;
//
//  // Convert from u (column / width), v (row/height) to position in array
//  // where X,Y,Z data starts
//  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;
//
//  // compute position in array where x,y,z data start
//  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
//  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
//  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
//
//  float X = 0.0;
//  float Y = 0.0;
//  float Z = 0.0;
//
//  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
//  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
//  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));
//
//  p.x = X;
//  p.y = Y;
//  p.z = Z;
//
//  std::cout << "z of centerpt_1" << Z << std::endl; //I don't know.
//
//}





int main(int argc, char **argv)
{


	ros::init(argc, argv, "hough_blue_bars");
	ros::NodeHandle nh;
	// cv::namedWindow("view");
	// cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback);

	ros::Subscriber subsCloud = nh.subscribe("camera/depth_registered/points", 1, callBackCloud);


	_pubWarped = it.advertise("warped", 1);
	_pubColorDetection = it.advertise("color_detected", 1);
	_pubMorphOperations = it.advertise("morph_operations", 1);
	_pubSkeleton = it.advertise("skeleton", 1);
	_pubHough = it.advertise("Hough_detection", 1);
	_pubCenter = it.advertise("Centerline", 1);


	_pubPath = nh.advertise<nav_msgs::Path>("path", 1);

	//_pubCanny = it.advertise("canny_filter", 1);
	//_pubThinned = it.advertise("thinned_image", 1);
	//_pubLines = it.advertise("lines", 1);

	dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig> server;
	dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();
	// cv::destroyWindow("view");
}



