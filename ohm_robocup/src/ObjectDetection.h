
#ifndef OBJECTDETECTION_H_
#define OBJECTDETECTION_H_


#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Image.h>


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace ohm{
struct object{
   unsigned int id;
   int x;
   int y;
   int w;
   int h;
   std::vector<cv::Point2f> p;
   cv::Point c;

   cv::Scalar color;
};
}

class ObjectDetection
{
private:    //dataelements
    ros::NodeHandle _nh;

    image_transport::ImageTransport it;

    ros::Publisher _pubImg;

    image_transport::Subscriber _subImgC;
    ros::Subscriber _subImg;
    ros::Subscriber _subObj;

    ros::Timer _loopTimer;

    sensor_msgs::Image _last_img;
    cv::Mat _img;

    std::map<unsigned int, std::string> _haz_texts;
    std::map<unsigned int, cv::Scalar> _haz_rect_colors;

    bool _rdy;


public:
    ObjectDetection();
    virtual ~ObjectDetection();

    /**
     *
     * @brief
     *
     * @return  void
     */
    void start(double duration = 0.01);

private:    //functions

    /**
     *
     * @brief this function containts the main working loop
     *
     * @param[in,out]  void
     *
     * @return 		   void
     */
    void run();


    void loop_callback(const ros::TimerEvent& e);

    //void subCallback(const ROS_PACK::MESSAGE& msg);

    void subImg_callback(const sensor_msgs::ImageConstPtr& msg);
    void subObj_callback(const std_msgs::Float32MultiArray& msg);

};

#endif /* OBJECTDETECTION_H_ */
