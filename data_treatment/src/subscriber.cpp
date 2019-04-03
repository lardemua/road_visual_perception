/**
 * @file subscriber.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief The data received from the algorithms is treated here!
 * @version 0.1
 * @date 2019-04-02
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <geometry_msgs/Point32.h>
#include <lane_detector/fitting.h>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std; // ja na e preciso usar o std
using namespace ros;
using namespace cv;
/**
 * @brief This callback is the one that receives the lanes
 * 
 */

// lane_detector::Lane current_lane_msg;
// std::vector<geometry_msgs::Point32::ConstPtr> point;

ros::Publisher lane_pub;
ros::Publisher initialimage;
ros::Publisher finalimage;
cv_bridge::CvImagePtr current_image;

void lanereceiveCallback(const lane_detector::Lane::ConstPtr &msg)
{
  std::vector<geometry_msgs::Point32> left;
  std::vector<geometry_msgs::Point32> right;
  std::vector<geometry_msgs::Point32> guide;
  // ROS_INFO_STREAM("Received pose: " << msg);
  left = msg->left_line;
  right = msg->right_line;
  guide = msg->guide_line;
  lane_pub.publish(msg);
}

/**
 * @brief Function that shows the polygon  on the image;
 * 
 */

void processImage()
{
  if (current_image)
  {
    int i = 0;
    int j = 0;
    cv::Mat image_bgr = current_image->image;
    cv::Mat image_rgb;
    cv::cvtColor(image_bgr, image_rgb, CV_BGR2RGB);
    auto top = 0;
    auto bottom = 0;
    auto left = 0;
    auto right = 0;
    int borderType;
    Scalar value;

    borderType = BORDER_CONSTANT;
    value = Scalar(0, 0, 0);
    copyMakeBorder(image_rgb, image_rgb, top, bottom, left, right, borderType, value);


    for (i = 0; i < image_rgb.rows; i++)
    {
      for (j = 0; j < image_rgb.cols; j++)
      {
        if ((image_rgb.at<Vec3b>(i, j)[1] == 255) && (image_rgb.at<Vec3b>(i, j)[0] == 0) && (image_rgb.at<Vec3b>(i, j)[2] == 0))
        {
          image_rgb.at<Vec3b>(i, j)[0] = 255;
          image_rgb.at<Vec3b>(i, j)[1] = 255; // green channel
          image_rgb.at<Vec3b>(i, j)[2] = 255;
        }

        if (i > 30 && i < image_rgb.rows - 30 && j > 25 && j < image_rgb.cols - 40)
        {
          //Pintar a área dentro da estrada a verde
          //Se eu for amarelo passo a verde
          if ((image_rgb.at<Vec3b>(i, j)[0] == 255) && (image_rgb.at<Vec3b>(i, j)[1] == 255) && (image_rgb.at<Vec3b>(i, j)[2] == 0))
          {
            image_rgb.at<Vec3b>(i, j)[0] = 0;
            image_rgb.at<Vec3b>(i, j)[1] = 255; // green channel
            image_rgb.at<Vec3b>(i, j)[2] = 0;
          }
          //Se eu for nao branco e do meu lado esquerdo for branco entao passo a verde
          if ((j < image_rgb.cols / 2) && ((image_rgb.at<Vec3b>(i, j)[0] != 255) && (image_rgb.at<Vec3b>(i, j)[1] != 255) && (image_rgb.at<Vec3b>(i, j)[2] != 255)) && ((image_rgb.at<Vec3b>(i, j - 1)[0] == 255) && (image_rgb.at<Vec3b>(i, j - 1)[1] == 255) && (image_rgb.at<Vec3b>(i, j - 1)[2] == 255)))
          {
            image_rgb.at<Vec3b>(i, j)[0] = 0;
            image_rgb.at<Vec3b>(i, j)[1] = 255; // green channel
            image_rgb.at<Vec3b>(i, j)[2] = 0;
          }
          //Se eu for diferente de branco e se do meu lado esquerdo e verde
          if (((image_rgb.at<Vec3b>(i, j)[0] != 255) || (image_rgb.at<Vec3b>(i, j)[1] != 255) || (image_rgb.at<Vec3b>(i, j)[2] != 255)) && ((image_rgb.at<Vec3b>(i, j - 1)[0] == 0) && (image_rgb.at<Vec3b>(i, j - 1)[1] == 255) && (image_rgb.at<Vec3b>(i, j - 1)[2] == 0))) //&& ((image_rgb.at<Vec3b>(i, j + 1)[0] != 255) && (image_rgb.at<Vec3b>(i, j + 1)[1] != 255) && (image_rgb.at<Vec3b>(i, j + 1)[2] != 255)))
          {
            image_rgb.at<Vec3b>(i, j)[0] = 0;
            image_rgb.at<Vec3b>(i, j)[1] = 255; // green channel
            image_rgb.at<Vec3b>(i, j)[2] = 0;
          }
        }
      }
    }
    cv::cvtColor(image_rgb, image_rgb, CV_RGB2BGR);
    auto processed_img = cv_bridge::CvImage{current_image->header, "bgr8", image_rgb};
    finalimage.publish(processed_img);
  }
}

/**
 * @brief This callback is the one that receives the processed image by the algorithms
 * 
 */

void imagereceiveCallback(const sensor_msgs::ImageConstPtr &img)
{
  try
  {
    // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    // cv::waitKey(30);
    current_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    initialimage.publish(img);
    processImage();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  ros::Subscriber sub_lanes = n.subscribe("lane_detector/lane", 1000, lanereceiveCallback);
  image_transport::Subscriber sub = it.subscribe("lane_detector/processed", 10, imagereceiveCallback);

  lane_pub = n.advertise<lane_detector::Lane>("estou_publicar/lane_received", 10);
  initialimage = n.advertise<sensor_msgs::Image>("data_treatment/initial", 10);
  finalimage = n.advertise<sensor_msgs::Image>("data_treatment/final", 10);

  ros::spin();
  return 0;
}