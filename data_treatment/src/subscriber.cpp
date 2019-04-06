/**
 * @file subscriber.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief The data received from each algorithms is treated here!
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

using namespace std;  // ja na e preciso usar o std
using namespace ros;
using namespace cv;

ros::Publisher lane_pub;
ros::Publisher initialimage;
ros::Publisher finalimage;
cv_bridge::CvImagePtr current_image;

/**
 * @brief Function that converts ROS points in cv float Points
 *
 * @param input ROS points
 * @return std::vector<cv::Point2f>
 */

inline std::vector<cv::Point2f> ROS2CvPoint2f(std::vector<geometry_msgs::Point32> &input)
{
  std::vector<cv::Point2f> output;
  for (geometry_msgs::Point32 point_32 : input)
  {
    cv::Point2f point;
    point.x = -point_32.y;
    point.y = point_32.x;
    output.push_back(point);
  }
  return output;
}

/**
 * @brief Function thar converts cv float Points in cv Points
 *
 * @param input cv float points
 * @return std::vector<cv::Point>
 */

inline std::vector<cv::Point> cvtCvPoint2f2CvPoint(const std::vector<cv::Point2f> &input)
{
  std::vector<cv::Point> output;
  for (cv::Point2f p_float : input)
  {
    cv::Point p;
    p.x = cvRound(p_float.x);
    p.y = cvRound(p_float.y);
    if (p.x > 0)
      output.push_back(p);
  }
  return output;
}

/**
 * @brief Get the Image Size object
 *
 * @return tuple<rows, columns>
 */

tuple<int, int> getImageSize()
{
  if (current_image)
  {
    Mat image_cv = current_image->image;

    float image_rows = image_cv.rows;
    float image_cols = image_cv.cols;
    return { image_rows, image_cols };
  }
  else
  {
    return { 0, 0 };
  }
}

/**
 * @brief Image that draw a spline into an image
 *
 * @param inImage- the image inpout
 * @param splinePoints - the spline points
 * @param thickness - the drawn line thickness
 * @param color - the color line
 */

inline void drawSpline(cv::Mat inImage, const std::vector<cv::Point> &splinePoints, const uint32_t &thickness,
                       const cv::Scalar &color)
{
  const cv::Point *pts = (const cv::Point *)cv::Mat(splinePoints).data;
  int npts = cv::Mat(splinePoints).rows;

  cv::polylines(inImage, &pts, &npts, 1,
                false,     // draw open contour
                color,     // colour RGB ordering (here = green)
                thickness  // line thickness
  );
}

/**
 * @brief This callback is the one that receives the lanes and show images with the filled area
 *
 */

void lanereceive_showimageCallback(const lane_detector::Lane::ConstPtr &msg)
{
  std::vector<geometry_msgs::Point32> left;
  std::vector<geometry_msgs::Point32> right;
  std::vector<geometry_msgs::Point32> guide;
  std::vector<geometry_msgs::Point32> left_pix;
  std::vector<geometry_msgs::Point32> right_pix;
  std::vector<geometry_msgs::Point32> guide_pix;
  int row_image = 0;
  int col_image = 0;
  int n, k = 0;
  int row_max_left_lane = 0;
  int row_min_left_lane = 10000;
  int row_max_right_lane = 0;
  int row_min_right_lane = 10000;
  int col_max_left_lane = 0;
  int col_max_right_lane = 0;
  int col_min_left_lane = 0;
  int col_min_right_lane = 0;
  int R_channel = 255;
  int G_channel = 255;
  int B_channel = 255;
  CvPoint min_ll;
  CvPoint min_rl;
  CvPoint max_ll;
  CvPoint max_rl;
  CvPoint init_fill;

  left = msg->left_line;
  right = msg->right_line;
  guide = msg->guide_line;
  left_pix = msg->left_line_pix;
  right_pix = msg->right_line_pix;
  guide_pix = msg->guide_line_pix;

  std::vector<cv::Point2f> left_spline_pix_float = ROS2CvPoint2f(left_pix);
  std::vector<cv::Point2f> right_spline_pix_float = ROS2CvPoint2f(right_pix);
  std::vector<cv::Point2f> guide_spline_pix_float = ROS2CvPoint2f(guide_pix);
  std::vector<cv::Point> left_spline = cvtCvPoint2f2CvPoint(left_spline_pix_float);
  std::vector<cv::Point> right_spline = cvtCvPoint2f2CvPoint(right_spline_pix_float);
  std::vector<cv::Point> guide_spline = cvtCvPoint2f2CvPoint(guide_spline_pix_float);

  std::tuple<float, float> imageSize = getImageSize();
  row_image = get<0>(imageSize);
  col_image = get<1>(imageSize);

  if (current_image && (left_spline.size() > 0) && (right_spline.size() > 0) && (guide_spline.size() > 0))
  {
    ros::param::get("~R_channel", R_channel);
    ros::param::get("~G_channel", G_channel);
    ros::param::get("~B_channel", B_channel);
    Mat processedImage = Mat::zeros(row_image, col_image, CV_8UC3);
    drawSpline(processedImage, left_spline, 1, Scalar(B_channel, G_channel, R_channel));
    drawSpline(processedImage, right_spline, 1, Scalar(B_channel, G_channel, R_channel));

    for (n = 0; n < left_spline.size(); n++)
    {
      if (left_spline[n].y < row_min_left_lane)
      {
        row_min_left_lane = left_spline[n].y;
        col_min_left_lane = left_spline[n].x;
        min_ll = left_spline[n];
      }
      if (left_spline[n].y > row_max_left_lane)
      {
        row_max_left_lane = left_spline[n].y;
        col_max_left_lane = left_spline[n].x;
        max_ll = left_spline[n];
      }
    }

    for (k = 0; k < right_spline.size(); k++)
    {
      if (right_spline[k].y < row_min_right_lane)
      {
        row_min_right_lane = right_spline[k].y;
        col_min_right_lane = right_spline[k].x;
        min_rl = right_spline[k];
      }
      if (right_spline[k].y > row_max_right_lane)
      {
        row_max_right_lane = right_spline[k].y;
        col_max_right_lane = right_spline[k].x;
        max_rl = right_spline[k];
      }
    }
    // Debugger
    // cout<< "Minimos:"<< endl;
    // cout << "Left lane: (row_min,col_min)= " << row_min_left_lane << "," << col_min_left_lane << endl;
    // cout << "Right lane: (row_min,col_min)= "<<row_min_right_lane<<","<<col_min_right_lane<<endl;

    // cout << "Maximos:"<<endl,
    // cout << "Left lane: (row_max,col_max)= "<<row_max_left_lane<<","<<col_max_left_lane<<endl;
    // cout <<"Right lane: (row_max_col_max)= "<<row_max_right_lane<<","<<col_max_right_lane<<endl;
    init_fill.x = min_ll.x + 1;
    init_fill.y = min_ll.y + 1;

    line(processedImage, min_ll, min_rl, Scalar(B_channel, G_channel, R_channel), 1);
    line(processedImage, max_ll, max_rl, Scalar(B_channel, G_channel, R_channel), 1);
    floodFill(processedImage, init_fill, Scalar(B_channel, G_channel, R_channel));
    // processedImage.convertTo(processedImage, CV_8UC3);

    auto processed_img = cv_bridge::CvImage{ current_image->header, "bgr8", processedImage };
    finalimage.publish(processed_img);
  }
  // lane_pub.publish(msg);
}

/**
 * @brief Function that shows the result
 *
 */

void processImage()
{
  if (current_image)
  {
    // for (i = 0; i < image_rgb.rows; i++)
    // {
    //   for (j = 0; j < image_rgb.cols; j++)
    //   {
    //     if ((image_rgb.at<Vec3b>(i, j)[1] == 255) && (image_rgb.at<Vec3b>(i, j)[0] == 0) &&
    //         (image_rgb.at<Vec3b>(i, j)[2] == 0))
    //     {
    //       image_rgb.at<Vec3b>(i, j)[0] = 0;
    //       image_rgb.at<Vec3b>(i, j)[1] = 0;  // green channel
    //       image_rgb.at<Vec3b>(i, j)[2] = 0;
    //     }
    //     else
    //     {
    //       image_rgb.at<Vec3b>(i, j)[0] = 0;
    //       image_rgb.at<Vec3b>(i, j)[1] = 0;  // green channel
    //       image_rgb.at<Vec3b>(i, j)[2] = 0;
    //     }
    //   }
    // }
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
  ros::Subscriber sub_lanes = n.subscribe("lane_detector/lane", 10, lanereceive_showimageCallback);
  image_transport::Subscriber sub = it.subscribe("lane_detector/processed", 10, imagereceiveCallback);

  // lane_pub = n.advertise<lane_detector::Lane>("estou_publicar/lane_received", 10);
  initialimage = n.advertise<sensor_msgs::Image>("data_treatment/initial", 10);
  finalimage = n.advertise<sensor_msgs::Image>("data_treatment/final", 10);

  ros::spin();
  return 0;
}