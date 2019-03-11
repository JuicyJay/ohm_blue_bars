/*
 * hough_blue_bars.cpp
 *
 *  Created on: Sep 12, 2018
 *      Author: ninahetterich
 */



#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
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
#include <opencv2/features2d.hpp>
#include <boost/type.hpp>

static image_transport::Publisher _pubColorDetection;
static image_transport::Publisher _pubMorphOperations;
static image_transport::Publisher _pubSkeleton;
static image_transport::Publisher _pubHough;
static image_transport::Publisher _pubCenter;
static ros::Publisher _pubPath;
static pcl::PointCloud<pcl::PointXYZ> _cloud;
static image_transport::Publisher _pubEndpoints;


int Hmax = 130;
int Hmin = 90;
int Smax = 255;
int Smin = 56;
int Vmax = 255;
int Vmin = 110;

int intersections = 500;

int sizeA = 0;
int sizeB = 0;
int pointA = 0;
int pointB = 0;

void localizePixel(const cv::Point& pixel, geometry_msgs::Point& point);

// parameters to change in dynamic reconfigure
void callback(ohm_blue_bars::BlueBarsCfgConfig& config, uint32_t level) {

	std::cout << __PRETTY_FUNCTION__ << std::endl;


	Hmax = config.Hmax;
	Hmin = config.Hmin;
	Smax = config.Smax;
	Smin = config.Smin;
	Vmax = config.Vmax;
	Vmin = config.Vmin;

	intersections = config.intersections;

	sizeA = config.sizeA;
	sizeB = config.sizeB;
	pointA = config.pointA;
	pointB = config.pointB;

}

// detecting blue color
void colordetection(const cv::Mat& input, cv::Mat& blueFilter) {

	cv::Mat input_hsv;
	cvtColor(input,input_hsv,CV_RGB2HSV);
	inRange(input_hsv, cv::Scalar(Hmin, Smin, Vmin), cv::Scalar(Hmax, Smax, Vmax), blueFilter);

}

// generating a clear contour with morphological operations
void morphoperations(const cv::Mat& blueFilter) {
	cv::Mat element = getStructuringElement(cv::MORPH_RECT,
	cv::Size(sizeA, sizeB), cv::Point(pointA, pointB));
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

// generate skeleton to find the centerlines
void skeleton(const cv::Mat& blueFilter, cv::Mat& thinned) {
	cv::ximgproc::thinning(blueFilter, thinned,
			cv::ximgproc::THINNING_ZHANGSUEN);

	unsigned int minHorizontal = 40;
	unsigned int maxHorizontal = 460;
	for (unsigned int i = 0; i < thinned.rows - 3; i++) {
		if ((i < minHorizontal) || (i > maxHorizontal)) {
			for (unsigned int j = 0; j < thinned.cols - 1; j++) {
				thinned.at<int>(i, j) = 0;
			}
		}
	}
}

// locating lines as infinite lines and find start and end points
void houghdetection(const cv::Mat& thinned, cv::Mat& input,
		std::vector<cv::Vec2f>& lines) {

	HoughLines(thinned, lines, 1, CV_PI / 180, intersections, 0, 0);

	// computing the start and end points
	for (size_t i = 0; i < lines.size(); i++) {

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
		line(input, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);

	}

}


bool maxYaxis(const cv::Point& a, const cv::Point& b) {
	return a.y > b.y;
}
bool minYaxis(const cv::Point& a, const cv::Point& b) {
	return a.y < b.y;
}

// generating bar contours and locate the maxima and minima points
void findEndpoints(const cv::Mat& blueFilter, std::vector<std::vector<cv::Point> >& contours, cv::Mat& contourpic, double& ptsContourmax1_Y, double& ptsContourmin1_Y)
{

	contourpic = blueFilter;

	std::vector < cv::Point > ptsContourmax1;
	std::vector < cv::Point > ptsContourmin1;

	std::vector < cv::Point > ptsContourmax2;
	std::vector < cv::Point > ptsContourmin2;

	std::vector < cv::Vec4i > hierarchy;

	// finding all contours
	findContours(contourpic, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	int largest_contour= 0;
	int largest_index = 0;

	int second_largest_contour = 0;
	int second_largest_index = 0;

	// finding largest and second largest contours
	for (int i = 0; i < contours.size(); i++) {

		if (contours[i].size() > largest_contour) {
			second_largest_contour = largest_contour;
			second_largest_index = largest_index;
			largest_contour = contours[i].size();
			largest_index = i;


			ptsContourmax1 = contours[i];
			std::sort(ptsContourmax1.begin(), ptsContourmax1.end(), maxYaxis);


			ptsContourmin1 = contours[i];
			std::sort(ptsContourmin1.begin(), ptsContourmin1.end(), minYaxis);

		} else if (contours[i].size() > second_largest_contour) {
			second_largest_contour = contours[i].size();
			second_largest_index = i;

			ptsContourmax2 = contours[i];
			std::sort(ptsContourmax2.begin(), ptsContourmax2.end(), maxYaxis);


			ptsContourmin2 = contours[i];
			std::sort(ptsContourmin2.begin(), ptsContourmin2.end(), minYaxis);



		}


	}

	cv::cvtColor(contourpic, contourpic, CV_GRAY2BGR);

	drawContours(contourpic, contours, largest_index, cv::Scalar(255, 255, 0), 2, 8, hierarchy); // yellow
	drawContours(contourpic, contours, second_largest_index, cv::Scalar(0, 255, 0), 2, 8, hierarchy); // green

	// finding contour maxima and minima
	if(ptsContourmax1.size())
	{
		cv::circle(contourpic, ptsContourmax1[0], 6, cv::Scalar(100, 149, 237), 2); // blue
		cv::circle(contourpic, ptsContourmin1[0], 6, cv::Scalar(138,  43, 226), 2); // purple
	}

	if(ptsContourmax2.size())
	{
		cv::circle(contourpic, ptsContourmax2[0], 6, cv::Scalar(238,  59, 59), 2); // red
		cv::circle(contourpic, ptsContourmin2[0], 6, cv::Scalar(255, 127, 36), 2); // orange
	}


	ptsContourmax1_Y = ptsContourmax1[0].y;
	ptsContourmin1_Y = ptsContourmin1[0].y;





}

// generating the centerline and cut it to bar length
void centerline(const std::vector<cv::Vec2f>& lines, cv::Mat& center,
		cv::Mat& input, cv::Point& centerpt_0, cv::Point& centerpt_1, double& ptsContourmax1_Y, double& ptsContourmin1_Y,  cv::Point& abs_centerCut0, cv::Point& abs_centerCut1)
{
	if (!lines.size()) {
		std::cout << __PRETTY_FUNCTION__ << " error! Found no line "
				<< std::endl;
		return;
	}
	if (lines.size() != 2) {
		std::cout << __PRETTY_FUNCTION__
				<< " error! Found invalid size of lines " << lines.size()
				<< " should be 2 " << std::endl;
		return;
	}

	// finding points of intersections with x-axis
	std::vector < Eigen::Vector2d > cutLineVector0;
	std::vector < Eigen::Vector2d > cutLineVector1;

	for (size_t i = 0; i < lines.size(); i++) {

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

		Eigen::Vector2d pt1Vec(pt1.x, pt1.y);
		Eigen::Vector2d pt2Vec(pt2.x, pt2.y);

		Straight2D linestoCut(pt1Vec, pt2Vec);

		cv::Mat binaryMat(input.size(), input.type());
		Straight2D xaxis(Eigen::Vector2d(0.0, 0.0),
		Eigen::Vector2d(static_cast<double>(binaryMat.cols), 0.0));
		Straight2D xEdge(
		Eigen::Vector2d(0.0, static_cast<double>(binaryMat.rows)),
		Eigen::Vector2d(static_cast<double>(binaryMat.cols),
						static_cast<double>(binaryMat.rows)));
		// upper point of intersection with x-axis
		Eigen::Vector2d cutLine0 = linestoCut.cut(xaxis);
		// lower point of intersection with x-axis
		Eigen::Vector2d cutLine1 = linestoCut.cut(xEdge);
		cutLineVector0.push_back(cutLine0);
		cutLineVector1.push_back(cutLine1);

		center = input;

		cv::Point var1(cutLine0(0), cutLine0(1));
		cv::Point var2(cutLine1(0), cutLine1(1));

		cv::circle(center, var1, 20.0, cv::Scalar(173, 255, 47), 3, 8); //grüner kreis
		cv::circle(center, var2, 20.0, cv::Scalar(255, 140, 0), 3, 8); //orangener kreis

	}

	double middle0 = 0.0;


	// upper and lower point of centerline
	for (unsigned int i = 0; i < cutLineVector0.size(); i++) {
		middle0 += cutLineVector0[i].x();
	}
	middle0 /= static_cast<double>(cutLineVector0.size());
	std::cout << __PRETTY_FUNCTION__ << " middle0 = " << middle0 << std::endl;
	double xaxis_min = 0.0;
	cv::Point abs_centerPoint0(middle0, xaxis_min);
	cv::circle(center, abs_centerPoint0, 20.0, cv::Scalar(100, 100, 255), 3, 8);
	centerpt_0 = abs_centerPoint0;

	double middle1 = 0.0;
	for (unsigned int i = 0; i < cutLineVector1.size(); i++) {
		middle1 += cutLineVector1[i].x();
	}
	middle1 /= static_cast<double>(cutLineVector1.size());
	std::cout << __PRETTY_FUNCTION__ << " middle1 = " << middle1 << std::endl;
	double xaxis_max = 479.0;
	cv::Point abs_centerPoint1(middle1, xaxis_max);
	cv::circle(center, abs_centerPoint1, 20.0, cv::Scalar(200, 100, 255), 3, 8);
	centerpt_1 = abs_centerPoint1;

	line(center, abs_centerPoint0, abs_centerPoint1, cv::Scalar(255, 255, 0), 3,
			CV_AA);


	// stripped-down centerline
	cv::LineIterator it(center, abs_centerPoint0, abs_centerPoint1, 8);

	double minMid_X = 0.0;
	double maxMid_X = 0.0;

	for(int i = 0; i < it.count; i++, ++it)
	{

		cv::Point pt= it.pos();
		if (pt.y == static_cast<int>(std::round(ptsContourmax1_Y)))
		{
			maxMid_X = pt.x;
		}
		else if (pt.y == static_cast<int>(std::round(ptsContourmin1_Y)))
		{
			minMid_X = pt.x;
		}
		else
			continue;

	}

	abs_centerCut0 = cv::Point(maxMid_X, ptsContourmax1_Y);
	cv::circle(center, abs_centerCut0, 20.0, cv::Scalar(140, 8, 140), 3, 8);

	abs_centerCut1 = cv::Point(minMid_X, ptsContourmin1_Y);
	cv::circle(center, abs_centerCut1, 20.0, cv::Scalar(17, 8, 140), 3, 8);

	line(center, abs_centerCut0, abs_centerCut1, cv::Scalar(255,0,0), 1, CV_AA);

}


// generating pointcloud
void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

	_cloud = cloud;
}

// transforming points from 2D in 3D
void localizePixel(const cv::Point& pixel, geometry_msgs::Point& point) {
	static tf::TransformListener listener;
	std::string targetFrame = "base_link";
	tf::StampedTransform tf;
	if (!listener.waitForTransform(targetFrame, _cloud.header.frame_id,
			ros::Time(0), ros::Duration(3.0))) {
		std::cout << __PRETTY_FUNCTION__ << " timeout waiting for transform "
				<< std::endl;
		return;
	}
	try {
		listener.lookupTransform(targetFrame, _cloud.header.frame_id,
				ros::Time(0), tf);
	} catch (tf::TransformException& ex) {
		ROS_ERROR("%s", ex.what());
		return;
	}
	std::cout << __PRETTY_FUNCTION__ << " pixel " << pixel.x << " " << pixel.y
			<< std::endl;
	const unsigned int idx = pixel.y * _cloud.width + pixel.x;
	if (idx >= _cloud.size()) {
		std::cout << __PRETTY_FUNCTION__ << " idx " << idx << " out of range "
				<< _cloud.size() << std::endl;
		return;
	}
	pcl::PointXYZ pointPCL = _cloud.points[idx];
	tf::Vector3 vec(pointPCL.x, pointPCL.y, pointPCL.z);
	tf::Vector3 vecTransformed = tf * vec;
	point.x = vecTransformed.x();
	point.y = vecTransformed.y();
	point.z = vecTransformed.z();

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	cv::Mat image;
	try {
		image = cv_bridge::toCvCopy(msg, "rgb8")->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
				msg->encoding.c_str());
		return;
	}

	cv::Mat input = image;
	cv::Mat blueFilter = input;
	colordetection(input, blueFilter);

	if (_pubColorDetection.getNumSubscribers()) {
		cv_bridge::CvImage cvImageColor;
		cvImageColor.image = blueFilter;

		cvImageColor.encoding = "mono8";
		sensor_msgs::ImagePtr imageRosColor = cvImageColor.toImageMsg();
		_pubColorDetection.publish(imageRosColor);
	}

	morphoperations(blueFilter);
	if (_pubMorphOperations.getNumSubscribers()) {
		cv_bridge::CvImage cvImageMorph;
		cvImageMorph.image = blueFilter;
		cvImageMorph.encoding = "mono8";
		sensor_msgs::ImagePtr imageRosMorph = cvImageMorph.toImageMsg();
		_pubMorphOperations.publish(imageRosMorph);
	}

	cv::Mat thinned;
	skeleton(blueFilter, thinned);
	if (_pubSkeleton.getNumSubscribers()) {
		cv_bridge::CvImage cvImageThinned;
		cvImageThinned.image = thinned;
		cvImageThinned.encoding = "mono8";
		sensor_msgs::ImagePtr imageRosThinned = cvImageThinned.toImageMsg();
		_pubSkeleton.publish(imageRosThinned);
	}

	std::vector < cv::Vec2f > lines;
	houghdetection(thinned, input, lines);
	std::cout << __PRETTY_FUNCTION__ << " found " << lines.size() << " lines "
			<< std::endl;
	if (_pubHough.getNumSubscribers()) {
		cv_bridge::CvImage cvImageHough;
		cvImageHough.image = input;
		cvImageHough.encoding = msg->encoding;
		sensor_msgs::ImagePtr imageRosHough = cvImageHough.toImageMsg();
		_pubHough.publish(imageRosHough);
	}

	cv::Mat center;
	cv::Point centerpt_0;
	cv::Point centerpt_1;
	double ptsContourmax1_Y;
	double ptsContourmin1_Y;

	std::vector < std::vector<cv::Point> > contours;
	cv::Mat contourpic;
	findEndpoints(blueFilter, contours, contourpic, ptsContourmax1_Y, ptsContourmin1_Y);
	if (_pubEndpoints.getNumSubscribers()) {
		cv_bridge::CvImage cvImageEndpoints;
		cvImageEndpoints.image = contourpic;
		cvImageEndpoints.encoding = msg->encoding;
		sensor_msgs::ImagePtr imageRosEndpoints = cvImageEndpoints.toImageMsg();
		_pubEndpoints.publish(imageRosEndpoints);
	}
	cv::Point pathpoint0;
	cv::Point pathpoint1;
	centerline(lines, center, input, centerpt_0, centerpt_1, ptsContourmax1_Y, ptsContourmin1_Y, pathpoint0, pathpoint1);
	if (_pubCenter.getNumSubscribers()) {
		cv_bridge::CvImage cvImageCenter;
		cvImageCenter.image = center;
		cvImageCenter.encoding = msg->encoding;
		sensor_msgs::ImagePtr imageRosCenter = cvImageCenter.toImageMsg();
		_pubCenter.publish(imageRosCenter);
	}

	// generating path with centerline points
	geometry_msgs::Point point;
	geometry_msgs::Point point2;
	localizePixel(pathpoint0, point);
	localizePixel(pathpoint1, point2);
	geometry_msgs::PoseStamped pose;
	geometry_msgs::PoseStamped pose2;
	pose.header.frame_id = "base_link";
	pose2.header.frame_id = "base_link";
	pose.pose.position = point;
	pose2.pose.position = point2;
	nav_msgs::Path path;
	path.header.frame_id = "base_link";
	path.poses.push_back(pose);
	path.poses.push_back(pose2);
	_pubPath.publish(path);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "hough_blue_bars");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1,
			imageCallback);

	ros::Subscriber subsCloud = nh.subscribe("camera/depth_registered/points",
			1, callBackCloud);

	_pubColorDetection = it.advertise("color_detected", 1);
	_pubMorphOperations = it.advertise("morph_operations", 1);
	_pubSkeleton = it.advertise("skeleton", 1);
	_pubHough = it.advertise("Hough_detection", 1);
	_pubCenter = it.advertise("Centerline", 1);
	_pubEndpoints = it.advertise("Endpoints", 1);
	_pubPath = nh.advertise < nav_msgs::Path > ("path", 1);


	dynamic_reconfigure::Server < ohm_blue_bars::BlueBarsCfgConfig > server;
	dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();
}

