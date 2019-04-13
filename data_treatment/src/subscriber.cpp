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

/*ROS*/
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include <image_transport/image_transport.h>

/*Para a mensagem*/
#include <lane_detector/fitting.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

/*OpenCV*/
#include <opencv/cv.h>
#include <opencv/highgui.h>

/*Namesapaces*/
using namespace std; // ja na e preciso usar o std
using namespace ros;
using namespace cv;

class get_lines
{
public:
  get_lines();
  void Looping();

private:
  ros::NodeHandle n;

  /*importante variables*/
  std::vector<geometry_msgs::Point32> left;
  std::vector<geometry_msgs::Point32> right;
  std::vector<geometry_msgs::Point32> guide;
  std::vector<geometry_msgs::Point32> left_pix;
  std::vector<geometry_msgs::Point32> right_pix;
  std::vector<cv::Point> right_spline;
  std::vector<cv::Point> left_spline;
  std::vector<cv::Point2f> left_spline_pix_float;
  std::vector<cv::Point2f> right_spline_pix_float;

  /*ROS*/
  image_transport::ImageTransport it;

  /*Images Messages*/
  sensor_msgs::ImagePtr processed_img;

  /*Pubs & Subs*/
  ros::Publisher lane_pub;
  ros::Publisher finalimage;
  ros::Subscriber sub_lanes;
  image_transport::Subscriber sub;

  /*Image Pointers Receives*/
  cv_bridge::CvImagePtr current_image;

  /*Functions*/
  void Publishers();
  void drawingPolygn();
  void lanereceiveCallback(const lane_detector::Lane::ConstPtr &msg);
  void imagereceiveCallback(const sensor_msgs::ImageConstPtr &img);
  tuple<int, int> getImageSize();
  inline void drawSpline(cv::Mat inImage, const std::vector<cv::Point> &splinePoints, const uint32_t &thickness, const cv::Scalar &color);
  inline std::vector<cv::Point2f> ROS2CvPoint2f(std::vector<geometry_msgs::Point32> &input);
  inline std::vector<cv::Point> cvtCvPoint2f2CvPoint(const std::vector<cv::Point2f> &input);
};

/**
 * @brief Get the lines::get lines object
 * 
 */
get_lines::get_lines() : it(n)
{
  finalimage = n.advertise<sensor_msgs::Image>("data_treatment/final", 10);

  sub = it.subscribe("lane_detector/result", 10, &get_lines::imagereceiveCallback, this);
  sub_lanes = n.subscribe("lane_detector/lane", 10, &get_lines::lanereceiveCallback, this);
}

/**
 * @brief Main Loop
 * 
 */

void get_lines::Looping()
{

  if (current_image && (left_spline.size() > 0) && (right_spline.size() > 0)) //&& (guide_spline.size() > 0))
  {

    drawingPolygn();
    Publishers();
  }
}

/**
 * @brief Publishers function
 *  
 * 
 */

void get_lines::Publishers()
{
  finalimage.publish(processed_img);
}

/**
 * @brief This callback is the one that receives the image from the algorithms
 *
 */

void get_lines::imagereceiveCallback(const sensor_msgs::ImageConstPtr &img)
{
  try
  {
    current_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
  }
}

/**
 * @brief Function that converts ROS points in cv float Points
 *
 * @param input ROS points
 * @return std::vector<cv::Point2f>
 */

inline std::vector<cv::Point2f> get_lines::ROS2CvPoint2f(std::vector<geometry_msgs::Point32> &input)
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

inline std::vector<cv::Point> get_lines::cvtCvPoint2f2CvPoint(const std::vector<cv::Point2f> &input)
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

tuple<int, int> get_lines::getImageSize()
{
  if (current_image)
  {
    Mat image_cv = current_image->image;

    float image_rows = image_cv.rows;
    float image_cols = image_cv.cols;
    return {image_rows, image_cols};
  }
  else
  {
    return {0, 0};
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

inline void get_lines::drawSpline(cv::Mat inImage, const std::vector<cv::Point> &splinePoints, const uint32_t &thickness, const cv::Scalar &color)
{
  const cv::Point *pts = (const cv::Point *)cv::Mat(splinePoints).data;
  int npts = cv::Mat(splinePoints).rows;

  cv::polylines(inImage, &pts, &npts, 1,
                false,    // draw open contour
                color,    // colour RGB ordering (here = green)
                thickness // line thickness
  );
}

/**
 * @brief Get the lines::lanereceiveCallback object
 * 
 * @param msg 
 */

void get_lines::lanereceiveCallback(const lane_detector::Lane::ConstPtr &msg)
{
  left = msg->left_line;
  right = msg->right_line;
  guide = msg->guide_line;
  left_pix = msg->left_line_pix;
  right_pix = msg->right_line_pix;
  left_spline_pix_float = ROS2CvPoint2f(left_pix);
  right_spline_pix_float = ROS2CvPoint2f(right_pix);
  left_spline = cvtCvPoint2f2CvPoint(left_spline_pix_float);
  right_spline = cvtCvPoint2f2CvPoint(right_spline_pix_float);
}
// std::vector<cv::Point> guide_spline = cvtCvPoint2f2CvPoint(guide_spline_pix_float);
/**
 * @brief Draw the polygn region 
 * 
 */

void get_lines::drawingPolygn()
{
  int row_image = 10;
  int col_image = 10;
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
  std::tuple<float, float> imageSize = getImageSize();
  row_image = get<0>(imageSize);
  col_image = get<1>(imageSize);

  // cout << "estou ca dentro" << endl;
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

  processed_img = cv_bridge::CvImage{current_image->header, "bgr8", processedImage}.toImageMsg();
}


/**
 * @brief This callback is the one that receives the lanes and show images with the filled area
 *
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  get_lines subs_lines;

  while (ros::ok())
  {
    subs_lines.Looping();
    ros::spinOnce();
  }

  return 0;
}