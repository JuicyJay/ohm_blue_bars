
#include "ObjectDetection.h"

ObjectDetection::ObjectDetection() : it(_nh)
{
    //rosParam
    ros::NodeHandle privNh("~");
    std::string string_val;
    double      double_val;
    int         int_val;
    bool        bool_val;

    privNh.param(         "string_val" ,    string_val,   std::string("string"));
    privNh.param<double>( "double_val" ,    double_val,   100.0);
    privNh.param<int>(    "int_val"    ,    int_val   ,   1.0);
    privNh.param<bool>(   "bool_val"   ,    bool_val  ,   true);


    //init publisher
    //_pub = _nh.advertise<std_msgs::Bool>("pub_name",1);
    _pubImg = _nh.advertise<sensor_msgs::Image>("image_hazmat", 1);


    //inti subscriber
    //_sub = _nh.subscribe("subname", 1, &Template::subCallback, this);
    _subImgC = it.subscribe("image_raw", 1, &ObjectDetection::subImg_callback, this);
    //_subImg = _nh.subscribe("image_raw", 1, &ObjectDetection::subImg_callback, this);
    _subObj = _nh.subscribe("objects", 1, &ObjectDetection::subObj_callback, this);

//    _haz_texts.insert(std::make_pair(9,std::string("9")));
//    _haz_texts.insert(std::make_pair(10,std::string("10")));
//    _haz_texts.insert(std::make_pair(11,std::string("11")));
//    _haz_texts.insert(std::make_pair(12,std::string("12")));
//    _haz_texts.insert(std::make_pair(13,std::string("13")));
//    _haz_texts.insert(std::make_pair(14,std::string("14")));
//    _haz_texts.insert(std::make_pair(15,std::string("15")));
//    _haz_texts.insert(std::make_pair(16,std::string("16")));
//    _haz_texts.insert(std::make_pair(8,std::string("8")));
//    _haz_texts.insert(std::make_pair(9,std::string("9")));
//    _haz_texts.insert(std::make_pair(10,std::string("10")));
//    _haz_texts.insert(std::make_pair(11,std::string("11")));
//    _haz_texts.insert(std::make_pair(12,std::string("12")));

    _haz_texts.insert(std::make_pair(0 ,std::string("")));
    _haz_texts.insert(std::make_pair(1 ,std::string("Infectious Substance      ")));
    _haz_texts.insert(std::make_pair(2 ,std::string("Inhalation Hazard         ")));
    _haz_texts.insert(std::make_pair(3 ,std::string("1.4 Explosive             ")));
    _haz_texts.insert(std::make_pair(4 ,std::string("Spontaneously Combustible ")));
    _haz_texts.insert(std::make_pair(5 ,std::string("Dangerous When Wet        ")));
    _haz_texts.insert(std::make_pair(6 ,std::string("Organic Peroxide          ")));
    _haz_texts.insert(std::make_pair(7 ,std::string("Radioactive II            ")));
    _haz_texts.insert(std::make_pair(8 ,std::string("Non-Flammable Gas         ")));
    _haz_texts.insert(std::make_pair(9 ,std::string("Oxidizer                  ")));
    _haz_texts.insert(std::make_pair(10,std::string("Corrosive                 ")));
    _haz_texts.insert(std::make_pair(11,std::string("Flammable Liquid          ")));
    _haz_texts.insert(std::make_pair(12,std::string("Flammable Solid           ")));

    //1)  Infectious Substance
    //2)  Inhalation Hazard
    //3)  1.4 Explosive
    //4)  Spontaneously Combustible
    //5)  Dangerous When Wet
    //6)  Organic Peroxide
    //7)  Radioactive II
    //8)  Non-Flammable Gas
    //9)  Oxidizer
    //10) Corrosive
    //11) Flammable Liquid
    //12) Flammable Solid


    _haz_rect_colors.insert(std::make_pair(0 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(1 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(2 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(3 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(4 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(5 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(6 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(7 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(8 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(9 ,  cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(10, cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(11, cv::Scalar()));
    _haz_rect_colors.insert(std::make_pair(12, cv::Scalar()));

    _rdy=false;

}

ObjectDetection::~ObjectDetection()
{
}

void ObjectDetection::start(double duration)
{
   //create timer
   //_loopTimer = _nh.createTimer(ros::Duration(duration), &ObjectDetection::loop_callback, this);
   this->run();
}

void ObjectDetection::run()
{
   ros::spin();
}



void ObjectDetection::loop_callback(const ros::TimerEvent& e)
{
   //do loop stuff here!!!

}


void ObjectDetection::subImg_callback(const sensor_msgs::ImageConstPtr& msg)
{
   //_last_img = msg;
   cv_bridge::CvImagePtr cv_ptr;
   try
   {
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
   }

   ROS_INFO("Get Image");

   cv_ptr->image.copyTo(_img);
   _rdy = true;
}

void ObjectDetection::subObj_callback(const std_msgs::Float32MultiArray& msg)
{
   //12 elements for 1 objects
   ROS_INFO("Num Objects: %d", (int)msg.data.size());


   std::vector<ohm::object> objs;

   //extract Objects
   for(unsigned int i=0; i<msg.data.size(); i+=12)
   {
      ROS_INFO("-- dings: %d, (w,h): (%d, %d)",(int)msg.data[i],(int)msg.data[i+1],(int)msg.data[i+2]);
      ohm::object tmp;
      tmp.id = (unsigned int)msg.data[i];



      //to x and y
      cv::Mat H = cv::Mat::zeros(3,3,CV_32F);


      H.at<float>(0) = msg.data[i+3];
      H.at<float>(1) = msg.data[i+4];
      H.at<float>(2) = msg.data[i+5];
      H.at<float>(3) = msg.data[i+6];
      H.at<float>(4) = msg.data[i+7];
      H.at<float>(5) = msg.data[i+8];
      H.at<float>(6) = msg.data[i+9];
      H.at<float>(7) = msg.data[i+10];
      H.at<float>(8) = msg.data[i+11];

      std::cout << "H: " << H << std::endl;


      tmp.w = (int)msg.data[i+1];
      tmp.h = (int)msg.data[i+2];

//      std::vector<cv::Point2f> corners(4);
//      std::vector<cv::Point2f> out(4);
//      corners[0] = cv::Point2f(0,0);
//      corners[1] = cv::Point2f(tmp.w,0);
//      corners[2] = cv::Point2f(tmp.w,tmp.h);
//      corners[3] = cv::Point2f(0,tmp.h);
//
//      std::cout << "corners: " << corners << std::endl;
//
//      cv::perspectiveTransform(corners, out, H);
//
//      tmp.p = out;
//
//
//      std::cout << "out: " << out << std::endl;
//
//      int xc = 0;
//      int yc = 0;
//      for(unsigned int i=0; i<4; ++i)
//      {
//         xc+=out[i].x;
//         yc+=out[i].y;
//      }
//      xc /= 4;
//      yc /= 4;

      //tmp.c.x = xc;
      //tmp.c.y = yc;

      tmp.c.x = H.at<float>(6);
      tmp.c.y = H.at<float>(7);;

      std::cout << "--------- center: " << tmp.c << std::endl;

      //tmp.x = (int)out[];
      //tmp.y = (int)msg.data[i+10];


      //tmp.x = H.at<float>(0)*x + m21*y + dx
      //tmp.y' = m22*y + m12*x + dy



      objs.push_back(tmp);
   }

   cv::Scalar tmp_color(255,0,0,0);

   //draw rects in cv_img:
   for(unsigned int i=0; i<objs.size(); ++i)
   {
      ROS_INFO("Draw stuff ");
      //draw rect
//      cv::Point p1(objs[i].x - objs[i].w, objs[i].y);
//      cv::Point p2(objs[i].x + objs[i].w, objs[i].y + objs[i].h);
//      cv::rectangle(_img,p1, p2,tmp_color,3);

      //draw Polygon
      //-- Draw lines between the corners
//      cv::line(_img, objs[i].p[0], objs[i].p[1], tmp_color, 3);
//      cv::line(_img, objs[i].p[1], objs[i].p[2], tmp_color, 3);
//      cv::line(_img, objs[i].p[2], objs[i].p[3], tmp_color, 3);
//      cv::line(_img, objs[i].p[3], objs[i].p[0], tmp_color, 3);

      cv::circle(_img, objs[i].c,40,tmp_color,6);


      //draw Text
      cv::putText(_img,_haz_texts.at(objs[i].id),objs[i].c,cv::FONT_HERSHEY_PLAIN, 2, tmp_color, 2);
   }

   ROS_INFO("RePub Img");

   //to ros_msg:
   cv_bridge::CvImage bridge_img;
   bridge_img.image = _img;
   //_pubImg.publish(bridge_img.toImageMsg());

   if(_rdy)
   {
      ROS_INFO("Scho img");
      cv::imshow("hallo", _img);
      cv::waitKey(5);
   }
   //_rdy = false;
}







// ------------- main ---------------
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "object_detection_node");
    ros::NodeHandle nh("~");

    ObjectDetection node;
    node.start();

}

