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

//#include <sensor_msgs/PointCloud2.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/point_types.h>

//static image_transport::Publisher _pubWarped;
static image_transport::Publisher _pubColorDetection;
static image_transport::Publisher _pubMorphOperations;
static image_transport::Publisher _pubSkeleton;
static image_transport::Publisher _pubHough;
static image_transport::Publisher _pubCenter;
static ros::Publisher _pubPath;
static pcl::PointCloud<pcl::PointXYZ> _cloud;
static image_transport::Publisher _pubEndpoints;

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
int Gmax = 125;
int Gmin = 0;

int intersections = 500;

int sizeA = 0;
int sizeB = 0;
int pointA = 0;
int pointB = 0;

void localizePixel(const cv::Point& pixel, geometry_msgs::Point& point);

void callback(ohm_blue_bars::BlueBarsCfgConfig& config, uint32_t level) {

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

//void warpimage(const cv::Mat& input, cv::Mat& warped) {
//	double f = 0.0;
//	double dist = 0.0;
//	double alpha;
//	double beta;
//	double gamma;
//
//	alpha = ((double) alpha_ - 90.) * M_PI / 180;
//	beta = ((double) beta_ - 90.) * M_PI / 180;
//	gamma = ((double) gamma_ - 90.) * M_PI / 180;
//	f = (double) f_;
//	dist = (double) dist_;
//	cv::Size taille = input.size();
//	double w = (double) taille.width, h = (double) taille.height;
//	cv::Mat A1 =
//			(cv::Mat_<float>(4, 3) << 1, 0, -w / 2, 0, 1, -h / 2, 0, 0, 0, 0, 0, 1);
//
//	// Rotation matrices around the X,Y,Z axis
//	cv::Mat RX =
//			(cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, std::cos(alpha), -std::sin(
//					alpha), 0, 0, std::sin(alpha), std::cos(alpha), 0, 0, 0, 0, 1);
//
//	cv::Mat RY =
//			(cv::Mat_<float>(4, 4) << std::cos(beta), 0, -std::sin(beta), 0, 0, 1, 0, 0, std::sin(
//					beta), 0, std::cos(beta), 0, 0, 0, 0, 1);
//
//	cv::Mat RZ =
//			(cv::Mat_<float>(4, 4) << std::cos(gamma), -std::sin(gamma), 0, 0, std::sin(
//					gamma), std::cos(gamma), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
//
//	//  Rotationsmatrix mit (RX,RY,RZ)
//	cv::Mat R = RX * RY * RZ;
//
//	// Translation matrix on the Z axis change dist will change the height
//	cv::Mat T =
//			(cv::Mat_<float>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, dist, 0, 0, 0, 1); // Camera Intrisecs matrix 3D -> 2D
//	cv::Mat A2 =
//			(cv::Mat_<float>(3, 4) << f, 0, w / 2, 0, 0, f, h / 2, 0, 0, 0, 1, 0);
//
//	// Final and overall transformation matrix
//	cv::Mat transfo = A2 * (T * (R * A1));
//
//	// Apply matrix transformation
//	warpPerspective(input, warped, transfo, taille,
//			cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
//
//	//std::cout << "Fehler 1" << std::endl;
//}

void colordetection(const cv::Mat& input, cv::Mat& blueFilter) {

	inRange(input, cv::Scalar(Bmin, Rmin, Gmin), cv::Scalar(Bmax, Rmax, Gmax),
			blueFilter);  // BRG

}

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

void houghdetection(const cv::Mat& thinned, cv::Mat& input,
		std::vector<cv::Vec2f>& lines) { //Vec2f -> 4i

	//std::cout << "pt 1 = (lalala) " << pt1 << std::endl;
	//std::cout << "Hough" << std::endl;

	//std::vector<cv::Vec2f> lines;
	HoughLines(thinned, lines, 1, CV_PI / 180, intersections, 0, 0);
	std::cout << __PRETTY_FUNCTION__ << " lines deteced " << lines.size()
					<< std::endl;
	//std::cout << cv::Pon

	for (size_t i = 0; i < lines.size(); i++) {

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
		line(input, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);

		//  std::cout << "pt1" << pt1 << "\n" <<"pt2" << pt2 << std::endl;
		std::cout << "What's in lines: " << lines[i] << std::endl;
		//  std::cout << "a:" << a << "\n" << "b:" << b << "\n" << "x0:" << x0 << "\n" << "y0:" << y0 << std::endl;
	}
	// std::cout << "pt 1 = (lalalaLUEEEE) " << pt1 << std::endl;
	//   std::cout << "Warped-type:" << warped.type() << std::endl;

}











bool maxYaxis(const cv::Point& a, const cv::Point& b) { //maximum der contour
	return a.y > b.y;
}
bool minYaxis(const cv::Point& a, const cv::Point& b) { // minimum der contour
	return a.y < b.y;
}

void findEndpoints(const cv::Mat& blueFilter, std::vector<std::vector<cv::Point> >& contours, cv::Mat& contourpic, double& ptsContourmax1_Y, double& ptsContourmin1_Y) //cv::Mat& endpoints, cv::Point& centerpt_0, cv::Point& centerpt_1) {
{

	////	// Endpunkte nur sehr schwer findbar wenn andere weiße Pixel vorhanden -> enorm unsicher
	////		std::vector<cv::Point> whitepixel;
	//////		cv::Mat whitepixel;
	////		cv::findNonZero(thinned, whitepixel);
	////		std::cout << __PRETTY_FUNCTION__ << "white pixel:" << whitepixel.size() << std::endl;
	////
	////		double minp;
	////		double maxp;
	////		cv::minMaxLoc(whitepixel, &minp, &maxp);
	////
	////		std::cout << __PRETTY_FUNCTION__ << "minp:" << minp << "maxp:" << maxp << std::endl;
	////
	////		endpoints = thinned;
	////
	////		cv::Point mist1(centerpt_0.x, minp);
	////		cv::Point mist2(centerpt_1.x, maxp);
	////
	////
	////		cv::circle(endpoints, mist1, 20.0, cv::Scalar(255, 0, 0), 3, 8); //roter kreis
	////		cv::circle(endpoints, mist2, 10.0, cv::Scalar(255, 0, 0), 3, 8);
	//
	//
	//// versuch: mit blob die balken vom rest zu trennen
	//
	//
	////for (int i = 0; i < keypoints.size(); i++) {
	//
	//cv::SimpleBlobDetector detector;
	//
	////std::vector<cv::KeyPoint> keypoints;
	//detector.detect( thinned, keypoints);
	// //   std::cout << __PRETTY_FUNCTION__ << " keypoints = " <<  keypoints.x << std::endl;
	////}
	//
	////std::cout << __PRETTY_FUNCTION__ << " keypoints = " << keypoints << std::endl;
	//
	//

	contourpic = blueFilter;

	std::vector < cv::Point > ptsContourmax1;
	std::vector < cv::Point > ptsContourmin1;

	std::vector < cv::Point > ptsContourmax2;
	std::vector < cv::Point > ptsContourmin2;

	std::vector < cv::Vec4i > hierarchy;

	findContours(contourpic, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0)); // was ist hierarchy?

	// was ist index in dem fall? -> nummer des gefundenen contour?

	int largest_contour= 0;
	int largest_index = 0;

	int second_largest_contour = 0;
	int second_largest_index = 0;

	//	cv::Rect bounding_rect;

	for (int i = 0; i < contours.size(); i++) {
		//  Find the area of contour
		//		double a = contourArea(contours[i], false);
		//		if (a > largest_area) {
		//			largest_area = a;
		//			std::cout << i << " area a: " << a << std::endl;
		//
		//			// Store the index of largest contour
		//			largest_contour_index = i;
		//			// Find the bounding rectangle for biggest contour
		//			bounding_rect = boundingRect(contours[i]);
		//
		//			std::cout << "contours: " << contours[i] << std::endl;
		//
		//		}


		if (contours[i].size() > largest_contour) {
			second_largest_contour = largest_contour;
			second_largest_index = largest_index;
			largest_contour = contours[i].size();
			largest_index = i;

			//			std::cout << "contours.size " << contours[i].size() << std::endl;
			std::cout << __PRETTY_FUNCTION__ << "largest_index " << largest_index << std::endl;

			////// spitze finden tescht
			ptsContourmax1 = contours[i];
			std::sort(ptsContourmax1.begin(), ptsContourmax1.end(), maxYaxis);


			ptsContourmin1 = contours[i];
			std::sort(ptsContourmin1.begin(), ptsContourmin1.end(), minYaxis);

			std::cout << "test1" << std::endl;
			//		    for( size_t i = 0; i< contours.size(); i++ )
			//		    {
			//
			//		        ptsContour= contours[i];
			//
			//		        std::sort( ptsContour.begin(), ptsContour.end(), SortbyYaxis );
			//		     // std::sort( ptsContour.begin(), ptsContour.end(), SortbyXaxis );
			//		    	std::cout << "test spitze" << std::endl;
			//		    }
			//			for(int i= 0; i < contours.size(); i++)
			//			{
			//			    for(int j= 0; j < contours[i].size();j++) // run until j < contours[i].size();
			//			    {
			//			        std::cout << "contour1" << contours[i][j] << std::endl; //do whatever
			//			    }
			//			}


			std::cout << "test2" << std::endl;

		} else if (contours[i].size() > second_largest_contour) {
			second_largest_contour = contours[i].size();
			second_largest_index = i;

			//			for(int i= 0; i < contours.size(); i++)
			//			{
			//			    for(int j= 0; j < contours[i].size();j++) // run until j < contours[i].size();
			//			    {
			//			        std::cout << "contour2" << contours[i][j] << std::endl; //do whatever
			//			    }
			//			}

			ptsContourmax2 = contours[i];
			std::sort(ptsContourmax2.begin(), ptsContourmax2.end(), maxYaxis);


			ptsContourmin2 = contours[i];
			std::sort(ptsContourmin2.begin(), ptsContourmin2.end(), minYaxis);


			std::cout << "test3" << std::endl;

		}


	}

	cv::cvtColor(contourpic, contourpic, CV_GRAY2BGR);

	drawContours(contourpic, contours, largest_index, cv::Scalar(255, 255, 0), 2, 8, hierarchy); // gelb -> größte
	drawContours(contourpic, contours, second_largest_index, cv::Scalar(0, 255, 0), 2, 8, hierarchy); // grün


	std::cout << "test4" << std::endl;

	// segmentation fault evtl bzw warnings:
	//	cv::circle(contourpic, ptsContourmax1[0], 6, cv::Scalar(255, 0, 0), 2);
	//	cv::circle(contourpic, ptsContourmin1[0], 6, cv::Scalar(255, 165, 0), 2);

	//// segmentation fault sicher:
	//	cv::circle(contourpic, ptsContourmax2[0], 6, cv::Scalar(255, 0, 100), 2);
	//	cv::circle(contourpic, ptsContourmin2[0], 6, cv::Scalar(255, 165, 100), 2);


	// phil:
	// jetzt ziemlich stabil wenn größte contour gut erkannt
	if(ptsContourmax1.size())
	{
		cv::circle(contourpic, ptsContourmax1[0], 6, cv::Scalar(100, 149, 237), 2); // blau
		cv::circle(contourpic, ptsContourmin1[0], 6, cv::Scalar(138,  43, 226), 2); // lila
	}

	// nicht immer auf grüner contour ...
	if(ptsContourmax2.size())
	{
		cv::circle(contourpic, ptsContourmax2[0], 6, cv::Scalar(238,  59, 59), 2); // rot
		cv::circle(contourpic, ptsContourmin2[0], 6, cv::Scalar(255, 127, 36), 2); // organge
	}


	ptsContourmax1_Y = ptsContourmax1[0].y;
	ptsContourmin1_Y = ptsContourmin1[0].y;





}

////hä?
//
//void geradengleichung(int points[2][2])
//{
//	float y2=points[1][1], y1 = points[0][1];
//	float x2=points[1][0], x1 = points[0][0];
//// Geradengleichung: y = ax + b
//
//	float a,b;
//
//	a = (y2 - y1) / (x2 - x1);
//	b = y2 - (a*x2);
//
//}
//


void centerline(const std::vector<cv::Vec2f>& lines, cv::Mat& center,
		cv::Mat& input, cv::Point& centerpt_0, cv::Point& centerpt_1, double& ptsContourmax1_Y, double& ptsContourmin1_Y,  cv::Point& abs_centerCut0, cv::Point& abs_centerCut1) //, int& ptmax, int& ptmin)
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

	// schnittpunkt der Linien mit X-Achsen

	std::vector < Eigen::Vector2d > cutLineVector0;
	std::vector < Eigen::Vector2d > cutLineVector1;

	for (size_t i = 0; i < lines.size(); i++) {

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

		cv::Mat binaryMat(input.size(), input.type());
		// Eigen::Vector3d vec;
		Straight2D xaxis(Eigen::Vector2d(0.0, 0.0),
				Eigen::Vector2d(static_cast<double>(binaryMat.cols), 0.0));
		Straight2D xEdge(
				Eigen::Vector2d(0.0, static_cast<double>(binaryMat.rows)),
				Eigen::Vector2d(static_cast<double>(binaryMat.cols),
						static_cast<double>(binaryMat.rows)));

		Eigen::Vector2d cutLine0 = linestoCut.cut(xaxis); //oberer Schnittpunkt mit X-Achse als Vector
		Eigen::Vector2d cutLine1 = linestoCut.cut(xEdge); //unterer Schnittpunkt mit X-Achse als Vector
		cutLineVector0.push_back(cutLine0);
		cutLineVector1.push_back(cutLine1);
		std::cout << "Schnittpunkt mit x-Achse oben: \n" << cutLine0[0] << " & "
				<< cutLine0[1] << std::endl;
		//std::cout << "Schnittpunkt mit x-Achse unten: \n" << cutLine1 << std::endl;

		center = input;

		cv::Point var1(cutLine0(0), cutLine0(1));  //Schnittpunkt oben als Point
		cv::Point var2(cutLine1(0), cutLine1(1)); //Schnittpunkt unten als Point

		cv::circle(center, var1, 20.0, cv::Scalar(173, 255, 47), 3, 8); //grüner kreis
		cv::circle(center, var2, 20.0, cv::Scalar(255, 140, 0), 3, 8); //orangener kreis

	}

	double middle0 = 0.0;
	for (unsigned int i = 0; i < cutLineVector0.size(); i++) {
		middle0 += cutLineVector0[i].x();
	}
	middle0 /= static_cast<double>(cutLineVector0.size());
	std::cout << __PRETTY_FUNCTION__ << " middle0 = " << middle0 << std::endl;
	double xaxis_min = 0.0;
	cv::Point abs_centerPoint0(middle0, xaxis_min);
	cv::circle(center, abs_centerPoint0, 20.0, cv::Scalar(100, 100, 255), 3, 8); //blauer kreis
	centerpt_0 = abs_centerPoint0;

	double middle1 = 0.0;
	for (unsigned int i = 0; i < cutLineVector1.size(); i++) {
		middle1 += cutLineVector1[i].x();
	}
	middle1 /= static_cast<double>(cutLineVector1.size());
	std::cout << __PRETTY_FUNCTION__ << " middle1 = " << middle1 << std::endl;
	double xaxis_max = 479.0;
	cv::Point abs_centerPoint1(middle1, xaxis_max);
	cv::circle(center, abs_centerPoint1, 20.0, cv::Scalar(200, 100, 255), 3, 8); //rosa kreis
	centerpt_1 = abs_centerPoint1;

	line(center, abs_centerPoint0, abs_centerPoint1, cv::Scalar(255, 255, 0), 3,
			CV_AA);



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
	//cv::Point
	abs_centerCut0 = cv::Point(maxMid_X, ptsContourmax1_Y);
	cv::circle(center, abs_centerCut0, 20.0, cv::Scalar(140, 8, 140), 3, 8); //lila

	//cv::Point
	abs_centerCut1 = cv::Point(minMid_X, ptsContourmin1_Y);
	cv::circle(center, abs_centerCut1, 20.0, cv::Scalar(17, 8, 140), 3, 8); //dunkelblau


	line(center, abs_centerCut0, abs_centerCut1, cv::Scalar(255,0,0), 1, CV_AA);

}


void callBackCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
	std::cout << __PRETTY_FUNCTION__ << " pcl w h " << cloud.width << " "
			<< cloud.height << std::endl;
	_cloud = cloud;
}

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
	std::cout << __PRETTY_FUNCTION__ << " vecPreTf " << vec.x() << " "
			<< vec.y() << " " << vec.z() << std::endl;
	tf::Vector3 vecTransformed = tf * vec;
	point.x = vecTransformed.x();
	point.y = vecTransformed.y();
	point.z = vecTransformed.z();
	std::cout << __PRETTY_FUNCTION__ << " pixel " << pixel << " point "
			<< vecTransformed.x() << " " << vecTransformed.y() << " "
			<< vecTransformed.z() << std::endl;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	//std::cout << "image Callback" << std::endl;
	cv::Mat image;
	try {
		image = cv_bridge::toCvCopy(msg, "rgb8")->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
				msg->encoding.c_str());
		return;
	}


	//	cv::Mat warped;
	//	warpimage(image, warped);
	//
	//	if (_pubWarped.getNumSubscribers()) {
	//		cv_bridge::CvImage cvImage;
	//		cvImage.image = warped;
	//		cvImage.encoding = msg->encoding;
	//		_pubWarped.publish(cvImage.toImageMsg());
	//	}

	//std::cout << "image Callback warped" << std::endl;
	cv::Mat input = image;
	cv::Mat blueFilter = input;  //warum = warped?
	colordetection(input, blueFilter);

	if (_pubColorDetection.getNumSubscribers()) {
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
	if (_pubMorphOperations.getNumSubscribers()) {
		cv_bridge::CvImage cvImageMorph;
		cvImageMorph.image = blueFilter;
		cvImageMorph.encoding = "mono8";  //msg->encoding;
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

	//    cv::Point pt1;
	//    cv::Point pt2;
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
	//std::cout << "pt 1 in callback " << pt1 << std::endl;
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
		//cvImageEndpoints.encoding = "mono8";
		sensor_msgs::ImagePtr imageRosEndpoints = cvImageEndpoints.toImageMsg();
		_pubEndpoints.publish(imageRosEndpoints);
	}
	cv::Point fuckingPoint0;
	cv::Point fuckingPoint1;
	centerline(lines, center, input, centerpt_0, centerpt_1, ptsContourmax1_Y, ptsContourmin1_Y, fuckingPoint0, fuckingPoint1); //centerpt_0/_1 notwendig?
	if (_pubCenter.getNumSubscribers()) {
		cv_bridge::CvImage cvImageCenter;
		cvImageCenter.image = center;
		cvImageCenter.encoding = msg->encoding;
		sensor_msgs::ImagePtr imageRosCenter = cvImageCenter.toImageMsg();
		_pubCenter.publish(imageRosCenter);
	}


	geometry_msgs::Point point;
	geometry_msgs::Point point2;
	localizePixel(fuckingPoint0, point);
	localizePixel(fuckingPoint1, point2);
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

int main(int argc, char **argv) {

	ros::init(argc, argv, "hough_blue_bars");
	ros::NodeHandle nh;
	// cv::namedWindow("view");
	// cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1,
			imageCallback);

	ros::Subscriber subsCloud = nh.subscribe("camera/depth_registered/points",
			1, callBackCloud);

	//_pubWarped = it.advertise("warped", 1);
	_pubColorDetection = it.advertise("color_detected", 1);
	_pubMorphOperations = it.advertise("morph_operations", 1);
	_pubSkeleton = it.advertise("skeleton", 1);
	_pubHough = it.advertise("Hough_detection", 1);
	_pubCenter = it.advertise("Centerline", 1);
	_pubEndpoints = it.advertise("Endpoints", 1);

	_pubPath = nh.advertise < nav_msgs::Path > ("path", 1);

	//_pubCanny = it.advertise("canny_filter", 1);
	//_pubThinned = it.advertise("thinned_image", 1);
	//_pubLines = it.advertise("lines", 1);

	dynamic_reconfigure::Server < ohm_blue_bars::BlueBarsCfgConfig > server;
	dynamic_reconfigure::Server<ohm_blue_bars::BlueBarsCfgConfig>::CallbackType f;

	f = boost::bind(&callback, _1, _2);
	server.setCallback(f);

	ros::spin();
	// cv::destroyWindow("view");
}

