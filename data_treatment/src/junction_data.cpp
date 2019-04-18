/**
 * @file junction_data.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief The junction of the 2 images treated with painted area and the probability map are made here!
 * @version 0.1
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */

/*Opencv*/
#include <opencv/cv.h>
#include <opencv/highgui.h>

/*Mensagens*/
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point32.h>

/*GridMap*/
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"

/*ROS*/
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

/*Namesapaces*/
using namespace std;
using namespace ros;
using namespace cv;

class junction_data
{
public:
  junction_data();
  void MainLoop();

private:
  ros::NodeHandle n;

  /*Important variables*/
  double alpha_x, pace;

  /*ROS*/
  grid_map::GridMap grid_road_GridMap;
  nav_msgs::OccupancyGrid roadmapGrid;
  image_transport::ImageTransport it;

  /*Messages*/
  nav_msgs::MapMetaData info;
  sensor_msgs::CameraInfo::_K_type k_matrix;

  /*Publishers && Subs*/
  ros::Publisher merged_image;
  ros::Publisher intersect_image;
  ros::Publisher non_intersect_image;
  ros::Publisher prob_map_image;
  ros::Publisher grid_road_map_pub;
  image_transport::Subscriber sub_img1;
  image_transport::Subscriber sub_img2;
  image_transport::Subscriber sub_advanced_algo;

  /*Images pointers reveive*/
  cv_bridge::CvImagePtr current_image_alg1;
  cv_bridge::CvImagePtr current_image_alg2;
  cv_bridge::CvImagePtr current_image_alg3;

  /*Images messages*/
  sensor_msgs::ImagePtr img_final_summed, img_final_diff, img_final_nao_int, img_final_map;

  /*Functions*/
  void Publishers();
  void imageAlg1(const sensor_msgs::ImageConstPtr &img1);
  void imageAlg2(const sensor_msgs::ImageConstPtr &img2);
  void advanced_algo(const sensor_msgs::ImageConstPtr &img3);
  void mergedImage();
  void diffImage();
  void probabilitiesMapImage(Mat &input, Mat &input2);
};

/**
 * @brief Construct a new junction data::junction data object
 * 
 */

junction_data::junction_data() : it(n)
{
  /*Getting the scale factor m/pix*/
  sensor_msgs::CameraInfoConstPtr CamInfo;
  CamInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/camera_info", n, ros::Duration(10));
  k_matrix = CamInfo->K;
  alpha_x = k_matrix[0];
  pace = 1 / alpha_x; // cada lado da celula correponde a este valor em m

  /*Publishers and Subscribers*/
  merged_image = n.advertise<sensor_msgs::Image>("junction_data/summed_img", 10);
  intersect_image = n.advertise<sensor_msgs::Image>("junction_data/diff_img", 10);
  non_intersect_image = n.advertise<sensor_msgs::Image>("junction_data/nonintersect_img", 10);
  prob_map_image = n.advertise<sensor_msgs::Image>("junction_data/prob_map", 10);
  grid_road_map_pub = n.advertise<nav_msgs::OccupancyGrid>("road_probabilities_map", 10, true);

  sub_img1 = it.subscribe("data_treatment/final", 10, &junction_data::imageAlg1, this);
  sub_img2 = it.subscribe("data_treatment2/final", 10, &junction_data::imageAlg2, this);
  sub_advanced_algo = it.subscribe("advanced_algorithm/greenMask", 10, &junction_data::advanced_algo, this);
}

/**
 * @brief main loop
 * 
 */

void junction_data::MainLoop()
{
  if (current_image_alg1 && current_image_alg2 && current_image_alg3)
  {

    mergedImage();
    diffImage();
    Publishers();
  }
}

/**
 * @brief Publishers function
 * 
 * @param i
 */
void junction_data::Publishers()
{
  merged_image.publish(img_final_summed);
  intersect_image.publish(img_final_diff);
  non_intersect_image.publish(img_final_nao_int);
  prob_map_image.publish(img_final_map);
  grid_road_map_pub.publish(roadmapGrid);
}

/**
   * @brief Callback to receive the image that cames from the algorithm 1
   *
   * @param img1
   */
void junction_data::imageAlg1(const sensor_msgs::ImageConstPtr &img1)
{
  try
  {
    current_image_alg1 = cv_bridge::toCvCopy(img1, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img1->encoding.c_str());
  }
}

/**
   * @brief Callback to receive the image that cames from the algorithm 2
   *
   * @param img2
   */

void junction_data::imageAlg2(const sensor_msgs::ImageConstPtr &img2)
{
  try
  {
    current_image_alg2 = cv_bridge::toCvCopy(img2, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img2->encoding.c_str());
  }
}

/**
 * @brief Function tha receives the image with green area representing road zone.
 * 
 * @param img3 
 */

void junction_data::advanced_algo(const sensor_msgs::ImageConstPtr &img3)
{
  try
  {
    current_image_alg3 = cv_bridge::toCvCopy(img3, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img3->encoding.c_str());
  }
}

/**
   * @brief Function that shows the 2 retuned color images on the same image
   *
   */

void junction_data::mergedImage()
{
  int i,j=0;
  if (current_image_alg1 && current_image_alg2 && current_image_alg3)
  {
    Mat img_alg1 = current_image_alg1->image;
    Mat img_alg2 = current_image_alg2->image;
    Mat img_alg3 = current_image_alg3->image; //Image that is painted! Any algorithm that returns painted areas do like that!
    Mat img_summed;
  

    add(img_alg1, img_alg2, img_summed);
    add(img_summed, img_alg3, img_summed);
    img_final_summed = cv_bridge::CvImage{current_image_alg1->header, "bgr8", img_summed}.toImageMsg();
  }
}

/**
   * @brief Function that returns the intersection and non intersection region.
   *
   */

void junction_data::diffImage()
{
  if (current_image_alg1 && current_image_alg2 && current_image_alg3)
  {
    Mat img_diff;
    Mat img_ninter;
    Mat img_alg1 = current_image_alg1->image;
    Mat img_alg2 = current_image_alg2->image;
    Mat img_alg3 = current_image_alg3->image;
    cvtColor(img_alg1, img_alg1, CV_BGR2GRAY);
    cvtColor(img_alg2, img_alg2, CV_BGR2GRAY);
    cvtColor(img_alg3, img_alg3, CV_BGR2GRAY);
    threshold(img_alg1, img_alg1, 0, 255, THRESH_BINARY | THRESH_OTSU);
    threshold(img_alg2, img_alg2, 0, 255, THRESH_BINARY | THRESH_OTSU);
    threshold(img_alg3, img_alg3, 0, 255, THRESH_BINARY | THRESH_OTSU);

    bitwise_and(img_alg1, img_alg2, img_diff);
    bitwise_and(img_diff, img_alg3, img_diff);

    //bitwise_xor(img_alg1, img_alg2, img_ninter);
    bitwise_xor( img_diff,img_alg3, img_ninter); // rever isto!!
    
    img_final_diff = cv_bridge::CvImage{current_image_alg1->header, "mono8", img_diff}.toImageMsg();
    img_final_nao_int = cv_bridge::CvImage{current_image_alg1->header, "mono8", img_ninter}.toImageMsg();

    probabilitiesMapImage(img_diff, img_ninter);
  }
}

/**
   * @brief Function tha builds the probability map on an image type.
   *
   * @param input intersection zone
   * @param input2 non intersected zone
   */

void junction_data::probabilitiesMapImage(Mat &input, Mat &input2)
{
  if (current_image_alg1 && current_image_alg2 && current_image_alg3)
  {
    Mat kernel;
    Mat img_filt;
    Mat img_final;
    Point anchor;
    double delta;
    int ddepth;
    int kernel_size;
    int x = 0;
    int y = 0;
    float pix_cinzentosf = 0;
    int value_filter = 0;
    float prob = 0.0;
    float prob_non_intersect = 0.0;
    int thresh_non_intersect = 1; // valore acrescentado para termos a probabilidade das secções que nao se intersectam
    kernel_size = 25;
    ros::param::get("~kernel_size", kernel_size);
    input.convertTo(input, CV_32F);
    input2.convertTo(input2, CV_32F);
    input2 = input2 / (float)(kernel_size + thresh_non_intersect);

    kernel = Mat::ones(kernel_size, kernel_size, CV_32F) / (float)(kernel_size * kernel_size);

    /// Initialize arguments for the filter
    anchor = Point(-1, -1);
    delta = 0;
    ddepth = -1;

    filter2D(input, img_filt, ddepth, kernel, anchor, delta, BORDER_DEFAULT);
    img_final = Mat::zeros(input.rows, input.cols, CV_8UC1);
    img_filt = img_filt + input2;

    for (x = 0; x < input.rows; x++)
    {
      for (y = 0; y < input.cols; y++)
      {
        value_filter = img_filt.at<uchar>(x, y);
        prob = value_filter / (float)255;
        pix_cinzentosf = prob * (float)255;
        img_final.at<uchar>(x, y) = (int)pix_cinzentosf;

        if (input2.at<uchar>(x, y) != 0)
        {
          prob_non_intersect = input2.at<uchar>(x, y) / (float)255;
        }

        // if (value_filter != 0)
        // {
        //   cout << "value filter= " << value_filter << endl;
        //   cout << "Probabilidade " << prob << endl;
        // }
      }
    }

    img_filt.convertTo(img_filt, CV_8UC1);
    img_final_map = cv_bridge::CvImage{current_image_alg1->header, "mono8", img_filt}.toImageMsg();

    info.height = input.cols;
    info.width = input.rows;
    info.resolution = pace;
    info.origin.position.x = 0;
    info.origin.position.y = -(info.height * pace / 2);
    info.origin.position.z = 0;
    grid_map::GridMapRosConverter::initializeFromImage(*img_final_map, pace, grid_road_GridMap);
    grid_map::GridMapRosConverter::addLayerFromImage(*img_final_map, "road probability map", grid_road_GridMap, 0, 255, 128);
    grid_map::GridMapRosConverter::toOccupancyGrid(grid_road_GridMap, "road probability map", 0, 255, roadmapGrid);
    roadmapGrid.info = info;
    roadmapGrid.header = current_image_alg1->header;
    roadmapGrid.header.frame_id = "/world";
    // cout << "Header: " << roadmapGrid.header << endl;
  }
}

/**
 * @brief Main function
 *
 * @param argc
 * @param argv
 * @return int
 */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "junction_data");
  junction_data data;

  while (ros::ok())
  {
    data.MainLoop();
    ros::spinOnce();
  }

  return 0;
}
