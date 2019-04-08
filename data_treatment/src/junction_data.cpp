/**
 * @file junction_data.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief The junction of the 2 images treated with painted area is made here!
 * @version 0.1
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point32.h>
#include <image_transport/image_transport.h>
#include <lane_detector/fitting.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/ros.h"

using namespace std;
using namespace ros;
using namespace cv;

ros::Publisher merged_image;

class junction_data
{
public:
  cv_bridge::CvImagePtr current_image_alg1;
  cv_bridge::CvImagePtr current_image_alg2;

  void imageAlg1(const sensor_msgs::ImageConstPtr& img1)
  {
    try
    {
      current_image_alg1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::BGR8);
      mergedImage();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img1->encoding.c_str());
    }
  }

  void imageAlg2(const sensor_msgs::ImageConstPtr& img2)
  {
    try
    {
      current_image_alg2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::BGR8);
      mergedImage();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img2->encoding.c_str());
    }
  }

  void mergedImage()
  {
    if (current_image_alg1)
    {
      Mat img_alg1 = current_image_alg1->image;
      Mat img_alg2 = current_image_alg2->image;
      Mat img_summed;

      add(img_alg1,img_alg2,img_summed);
      auto img_final = cv_bridge::CvImage{ current_image_alg1->header, "bgr8", img_summed };
      merged_image.publish(img_final);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "junction_data");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  junction_data data;

  image_transport::Subscriber sub_img1 = it.subscribe("data_treatment/final", 10, &junction_data::imageAlg1, &data);
  image_transport::Subscriber sub_img2 = it.subscribe("data_treatment2/final", 10, &junction_data::imageAlg2, &data);
  merged_image = n.advertise<sensor_msgs::Image>("junction_data/summed_img", 10);
  ros::spin();

  return 0;
}
