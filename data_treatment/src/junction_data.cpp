/**
 * @file junction_data.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief Combine muitple algorithms' outputs
 * @version 0.1
 * @date 2019-04-02
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>

#include "opencv2/calib3d/calib3d.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>

/*Mensagens*/
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

/*GridMap*/
#include "grid_map_cv/GridMapCvConverter.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "std_msgs/Header.h"

/*ROS*/
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <tf/transform_listener.h>

/*write a file*/
#include <fstream>
#include <iostream>

struct image_info {
  cv::Mat image;
  bool isupdate;
  double image_delay;
};

/**
 * @brief Function that enables to write the result in a csv file
 *
 */

void writeResults2CSVfile(std::string frame_id, int sequence,
                          float area_ponderada, float area_comum,
                          float area_tot, float indicador1, float indicador2,
                          int kernel_size) {
  std::ofstream output;
  const std::string outFileName = "1_cam_2_algs_3_3.csv";
  output.open(outFileName, std::ios::app);

  output << frame_id << "," << sequence << "," << area_ponderada << ","
         << area_comum << "," << area_tot << "," << indicador1 << ","
         << indicador2 << "," << kernel_size << std::endl;
  output.close();
}

class junction_data {
public:
  struct params {
    int width;
    int height;
    int kernel_size;
    bool combine_cams;
    float f_x_camera;
  };
  /*Important variables*/
  double fx, pace, scale_factor, cols_img_big, cols_img_small;
  int physical_length;

  junction_data(const std::vector<std::string> &image_topics, params params);

  void img_callback(const sensor_msgs::ImageConstPtr &img1, int alg_idx);
  void process(int alg_idx);
  void probabilitiesMapImage(cv::Mat &img_intersection,
                             cv::Mat &img_nonintersection);

private:
  params _params;

  ros::NodeHandle _nh;
  image_transport::ImageTransport _it;
  std::vector<image_transport::Subscriber> _image_subscribers;
  std::vector<image_info> images_data_vect;
  ros::Publisher _image_publisher_summed;
  ros::Publisher _image_publisher_diff;
  ros::Publisher _image_publisher_nonint;
  ros::Publisher _prob_map_image;
  ros::Publisher _grid_road_map_pub;

  std::vector<std::string> _algorithms_names;
  std::vector<bool> _image_updates;
  std::vector<std::shared_ptr<cv::Mat>> _images;
  std::vector<std_msgs::Header> _images_headers;
  std::vector<double> _images_durations;

  /*Messages*/
  nav_msgs::MapMetaData info;
  sensor_msgs::CameraInfo::_K_type k_matrix;

  /*ROS*/
  grid_map::GridMap grid_road_GridMap;
  nav_msgs::OccupancyGrid roadmapGrid;

  /*Time variables*/
  ros::Time t_img;
  ros::Duration duration_image;
};

/**
 * @brief Construct a new junction data::junction data object
 *
 */

junction_data::junction_data(const std::vector<std::string> &image_topics,
                             params params)
    // clang-format off
    : _params(params)
    , _nh()
    , _it(_nh)
    , _image_publisher_summed(_nh.advertise<sensor_msgs::Image>("calc_prob_map/summed_img", 10))
    , _image_publisher_diff(_nh.advertise<sensor_msgs::Image>("calc_prob_map/intersect_img", 10))
    , _image_publisher_nonint(_nh.advertise<sensor_msgs::Image>("calc_prob_map/nonintersect_img", 10))
    , _prob_map_image(_nh.advertise<sensor_msgs::Image>("calc_prob_map/image_map", 10))
    , _grid_road_map_pub(_nh.advertise<nav_msgs::OccupancyGrid>("calc_prob_map/road_probabilistic_map", 10, true))
    , _algorithms_names(image_topics.begin(), image_topics.end())
    , _image_updates(image_topics.size(), false)
    , _images_headers(image_topics.size())
// clang-format on
{
  cols_img_big = 0;
  cols_img_small = 0;
  ros::param::get("~cols_img_big", cols_img_big);
  ros::param::get("~cols_img_small", cols_img_small);
  sensor_msgs::CameraInfoConstPtr CamInfo;
  CamInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
      "/camera/camera_info", _nh /*, ros::Duration(10)*/);
  k_matrix = CamInfo->K;
  fx = k_matrix[0];
  _params.f_x_camera = fx; // pixel/mm
  pace = (1 / (fx)) *
         (cols_img_big /
          cols_img_small); // cada lado da celula correponde a este valor em m

  for (auto topic = image_topics.begin(); topic != image_topics.end();
       topic++) {
    auto topic_idx = std::distance(image_topics.begin(), topic);

    auto callback =
        boost::bind(&junction_data::img_callback, this, _1, topic_idx);

    auto subscriber = _it.subscribe(*topic, 10, callback);

    _image_subscribers.push_back(subscriber);
  }

  _images.reserve(image_topics.size());
  for (int i = 0; i < image_topics.size(); i++) {
    auto img =
        std::make_shared<cv::Mat>(_params.height, _params.width, CV_8UC3);
    _images.push_back(img);
  }
}

/**
 * @brief Function that receives each image
 *
 * @param img_msg image messase
 * @param alg_idx algorithm index
 */

void junction_data::img_callback(const sensor_msgs::ImageConstPtr &img_msg,
                                 int alg_idx) {
  auto img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

  image_info indiv_img_data; // each image data
  // auto buffer = _images[alg_idx];
  img->image.copyTo(indiv_img_data.image);

  _image_updates[alg_idx] = true;
  indiv_img_data.image_delay = true;
  _images_headers[alg_idx] = img_msg->header;

  t_img = img_msg->header.stamp;
  duration_image = ros::Time::now() - t_img;
  _images_durations.push_back(duration_image.toSec());
  indiv_img_data.image_delay = _images_durations.back();
  // ROS_INFO_STREAM("Time to process: " << duration_image.toSec());

  // ROS_INFO("received image from algorithm %d", alg_idx);
  images_data_vect.push_back(indiv_img_data);
  process(alg_idx);
}

/**
 * @brief Intersection and disjunction of the images
 *
 */

void junction_data::process(int alg_idx) {
  cv::Mat image_int;
  cv::Mat image_ref;
  cv::Mat image_nonint;
  cv::Mat perspectiveMatrix;
  int rect_side, center_image, distance_between_lines, dist_between_cams_pix,
      lim_min, lim_max;

  float xs_zs, alt_virtual_cam, dist_between_cams;
  double delta_timer;
  double time_seconds;
  // verificar se as 3 images estão set
  for (auto is_updated : _image_updates) {
    if (!is_updated) {
      return;
    }
  }

  auto buffer_sum = images_data_vect.at(0).image;
  image_int = images_data_vect.at(0).image;

  cv::cvtColor(image_int, image_int, CV_BGR2GRAY);
  cv::threshold(image_int, image_int, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU);

  if (_params.combine_cams == true) {

  // Calculation side IPM-----------------------------------------
  dist_between_cams = 215;
  distance_between_lines = 3500; // mm
  alt_virtual_cam = 5794.04;
  xs_zs = (float)(distance_between_lines / alt_virtual_cam);
  rect_side = (_params.f_x_camera * xs_zs);

  if (_images_headers[alg_idx].frame_id == "top_left_camera") {
    dist_between_cams_pix =
        _params.f_x_camera * ((float)(dist_between_cams / alt_virtual_cam));
    center_image = _params.width / 2 - dist_between_cams_pix;

  } else if (_images_headers[alg_idx].frame_id == "top_right_camera") {
    center_image = _params.width / 2;
  }
  lim_min = (center_image - rect_side / 2);
  lim_max = (center_image + rect_side / 2);
  //-------------------------------------------------------------

  cv::Point2f perspectiveSrc[] = {cv::Point2f(370, 412), cv::Point2f(535, 412),
                                  cv::Point2f(88, 700), cv::Point2f(858, 700)};

  cv::Point2f perspectiveDst[] = {cv::Point2f(lim_min, 0),
                                  cv::Point2f(lim_max, 0),
                                  cv::Point2f(lim_min, _params.height),
                                  cv::Point2f(lim_max, _params.height)};

  ros::param::get("/top_right_camera/calc_prob_map_node/cams_combination",
                  _params.combine_cams);

  perspectiveMatrix =
      cv::getPerspectiveTransform(perspectiveSrc, perspectiveDst);
  cv::warpPerspective(image_int, image_int, perspectiveMatrix,
                      image_int.size());
  }
  for (int i = 1; i < images_data_vect.size(); i++) {
    // Too many time of processing? Don't count to the probabilistic map
    // if (images_data_vect.at(i).image_delay > 0.6)
    // {
    //     images_data_vect.erase(images_data_vect.begin()+i);
    //     continue;
    // }
    cv::add(buffer_sum, images_data_vect.at(i).image, buffer_sum);
    image_ref = images_data_vect.at(i).image;
    if (_params.combine_cams == true) {

      cv::warpPerspective(image_ref, image_ref, perspectiveMatrix,
                          image_ref.size());
    }
    cv::cvtColor(image_ref, image_ref, CV_BGR2GRAY);
    cv::threshold(image_ref, image_ref, 0, 255,
                  cv::THRESH_BINARY | cv::THRESH_OTSU);
    cv::bitwise_and(image_int, image_ref, image_int);
    cv::bitwise_xor(image_int, image_ref, image_nonint);
  }

  auto img_msg =
      cv_bridge::CvImage{_images_headers[0], "bgr8", buffer_sum}.toImageMsg();
  auto img_msg_diff =
      cv_bridge::CvImage{_images_headers[0], "mono8", image_int}.toImageMsg();
  auto img_msg_nonint =
      cv_bridge::CvImage{_images_headers[0], "mono8", image_nonint}
          .toImageMsg();

  _image_publisher_summed.publish(img_msg);
  _image_publisher_diff.publish(img_msg_diff);
  _image_publisher_nonint.publish(img_msg_nonint);

  probabilitiesMapImage(image_int, image_nonint);

  std::fill(_image_updates.begin(), _image_updates.end(), false);
  std::vector<double>().swap(_images_durations); // Delete and deallocation of
                                                 // the vector _images_durations
  std::vector<image_info>().swap(
      images_data_vect); // Delete and deallocation of the vector of structures
                         // _images_data_vect
}

/**
 * @brief Function tha builds the probability map on an image type.
 *
 * @param mage_intersection intersection zone
 * @param image_nonintersection non intersected zone
 */

void junction_data::probabilitiesMapImage(cv::Mat &image_intersection,
                                          cv::Mat &image_nonintersection) {

  cv::Size size(_params.width, _params.height);
  cv::Mat kernel;
  cv::Mat img_filt;
  cv::Mat img_final;
  cv::Mat img_original;
  cv::Mat final_result;
  cv::Point anchor;
  double delta;
  int ddepth;
  int x = 0;
  int y = 0;
  float pix_cinzentosf = 0;
  float prob = 0.0;
  float prob_non_intersect = 0.0;
  int thresh_non_intersect =
      0; // valor subtraído para termos a probabilidade das secções que nao
         // se intersectam (threshold)
  float area_common = 0;
  bool write_results_right = 0;
  bool write_results_left = 0;
  int least_probabilit = 0;
  int remainder = 0;
  float least_probability = 0;
  int kernel_size = 0;
  /// Initialize arguments for the filter
  anchor = cv::Point(-1, -1);
  delta = 0;
  ddepth = -1;

  ros::param::get("/dynamicParameters/kernel_size", _params.kernel_size);
  least_probabilit = _params.kernel_size / 2;
  remainder = _params.kernel_size % 2;
  kernel_size = _params.kernel_size * _params.kernel_size;
  least_probability =
      (float)(least_probabilit + remainder) / (float)(kernel_size);

  image_intersection.convertTo(image_intersection, CV_32F);
  image_nonintersection.convertTo(image_nonintersection, CV_32F);
  image_nonintersection =
      image_nonintersection * (float)(least_probability - thresh_non_intersect);

  kernel = cv::Mat::ones(_params.kernel_size, _params.kernel_size, CV_32F) /
           (float)(_params.kernel_size * _params.kernel_size);

  cv::filter2D(image_intersection, img_filt, ddepth, kernel, anchor, delta,
               cv::BORDER_DEFAULT);
  // img_final = cv::Mat::zeros(image_intersection.rows,
  // image_intersection.cols, CV_8UC1);
  img_filt = img_filt + image_nonintersection;

  img_filt.convertTo(img_filt, CV_8UC1);

  //-----------------------------------------------------------------------------------------------------
  //-------------------------------------------------------------------
  // Writing to a file
  ros::param::get("/top_right_camera/calc_prob_map_node/writing_results",
                  write_results_right);
  ros::param::get("/top_left_camera/calc_prob_map_node/writing_results",
                  write_results_left);

  if (write_results_right == true || write_results_left == true) {
    // Probabilities calculations:
    auto area_probabilistica_ponderada = cv::sum(img_filt)[0] / (float)255;
    auto area_total = cv::countNonZero(img_filt);

    for (x = 0; x < img_filt.rows; x++) {
      for (y = 0; y < img_filt.cols; y++) {

        if (img_filt.at<uchar>(x, y) == 255) {
          area_common++;
        }
      }
    }

    // Cálculos dos indicadores:
    auto I_1 = area_probabilistica_ponderada / area_total;
    auto I_2 = area_common / area_total;

    writeResults2CSVfile(_images_headers[0].frame_id, _images_headers[0].seq,
                         area_probabilistica_ponderada, area_common, area_total,
                         I_1, I_2, _params.kernel_size);
  }

  // GridMap:-----------------------------------------------------
  auto img_final_map =
      cv_bridge::CvImage{_images_headers[0], "mono8", img_filt}.toImageMsg();
  _prob_map_image.publish(img_final_map);

  info.height = image_intersection.cols;
  info.width = image_intersection.rows;
  info.resolution = pace;
  info.origin.position.x = 0;
  info.origin.position.y = -(info.height * pace / 2);
  info.origin.position.z = 0;
  grid_map::GridMapRosConverter::initializeFromImage(*img_final_map, pace,
                                                     grid_road_GridMap);
  grid_map::GridMapRosConverter::addLayerFromImage(
      *img_final_map, "road probability map", grid_road_GridMap, 0, 255, 128);
  grid_map::GridMapRosConverter::toOccupancyGrid(
      grid_road_GridMap, "road probability map", 0, 255, roadmapGrid);
  roadmapGrid.info = info;
  roadmapGrid.header = _images_headers[0];
  roadmapGrid.header.frame_id = "/world";
  _grid_road_map_pub.publish(roadmapGrid);
  // cout << "Header: " << roadmapGrid.header << endl;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "calc_prob_map_node");
  auto params = junction_data::params{};

  ros::param::get("~cols_img_small", params.width);
  ros::param::get("~rows_img_small", params.height);
  std::vector<std::string> topic_names;
  std::string topic_names_str;
  ros::param::get("~topics_polygons", topic_names_str);
  boost::split(topic_names, topic_names_str, boost::is_any_of(","));
  auto data_merger = junction_data(topic_names, params);

  ros::spin();

  return 0;
}
