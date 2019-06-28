/**
 * @file two_cameras_combination.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief Combine muitple cameras' outputs
 * @version 0.1
 * @date 2019-06-21
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <functional>
#include <memory>
#include <mutex>
#include <vector>
/*boost*/
#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>

/*ROS*/
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"
#include "tf_conversions/tf_eigen.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/eigen.hpp>
#include <tf/transform_listener.h>

/*write a file*/
#include <fstream>
#include <iostream>


/**
 * @brief Function that enables to write the result in a csv file
 *
 */

void writeResults2CSVfile(int sequence,
                          float area_ponderada, float area_comum,
                          float area_tot, float indicador1, float indicador2,
                          int kernel_size) {
  std::ofstream output;
  const std::string outFileName = "2_cam_2algs_51_51.csv";
  output.open(outFileName, std::ios::app);

  output << sequence << "," << area_ponderada << ","
         << area_comum << "," << area_tot << "," << indicador1 << ","
         << indicador2 << "," << kernel_size << std::endl;
  output.close();
}



class combineMaps {
public:
  struct params {
    int width;
    int height;
    int kernel_size;
  };
  combineMaps(std::vector<std::string> &maps_topics, params params);
  void mapCallback(const sensor_msgs::ImageConstPtr &map1, int map_idx);
  void warpMap(int map_idx);
  void buildMap();

private:
  params _params;
  ros::NodeHandle _nh;
  ros::Publisher _image_publisher_map1;
  ros::Publisher _image_publisher_map2;
  ros::Publisher _warp_map1;
  ros::Publisher _warp_map2;
  ros::Publisher _image_map;
  image_transport::ImageTransport _it;
  tf::TransformListener _tf_listener;

  std::vector<image_transport::Subscriber> _image_subscribers;
  std::vector<std::shared_ptr<cv::Mat>> _images;
  std::vector<std::shared_ptr<cv::Mat>> _warped_images;
  std::vector<std_msgs::Header> _maps_headers;
  std::vector<tf::StampedTransform> _transformations_vector;
  std::vector<bool> _maps_updates;
};

/**
 * @ Class Constructor
 *
 *
 *
 *
 */
combineMaps::combineMaps(std::vector<std::string> &maps_topics, params params)
    : _params(params), _it(_nh),
      _image_publisher_map1(_nh.advertise<sensor_msgs::Image>(
          "combine_multi_cams/initial_map1", 10)),
      _image_publisher_map2(_nh.advertise<sensor_msgs::Image>(
          "combine_multi_cams/initial_map2", 10)),
      _image_map(_nh.advertise<sensor_msgs::Image>(
          "combine_multi_cams/final_map", 10)),
      _warp_map1(
          _nh.advertise<sensor_msgs::Image>("combine_multi_cams/warp1", 10)),
      _warp_map2(
          _nh.advertise<sensor_msgs::Image>("combine_multi_cams/warp2", 10)),
      _maps_headers(maps_topics.size()),
      _transformations_vector(maps_topics.size()),
      _maps_updates(maps_topics.size(), false) {
  for (auto topic = maps_topics.begin(); topic != maps_topics.end(); topic++) {
    auto map_idx = std::distance(maps_topics.begin(), topic);
    auto callback = boost::bind(&combineMaps::mapCallback, this, _1, map_idx);
    auto subscriber = _it.subscribe(*topic, 10, callback);
    _image_subscribers.push_back(subscriber);
  }
  _images.reserve(maps_topics.size());
  _warped_images.reserve(maps_topics.size());
  for (int i = 0; i < maps_topics.size(); i++) {
    auto img =
        std::make_shared<cv::Mat>(_params.height, _params.width, CV_8UC1);
    auto warped_img =
        std::make_shared<cv::Mat>(_params.height, _params.width, CV_8UC1);
    _images.push_back(img);
    _warped_images.push_back(warped_img);
  }
}

/**
 * @brief Function that receives each map
 *
 * @param img_msg image messase
 * @param alg_idx algorithm index
 */

void combineMaps::mapCallback(const sensor_msgs::ImageConstPtr &img_msg,
                              int map_idx) {
  tf::StampedTransform transform;
  auto img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  ROS_INFO("received image from map %d", map_idx);
  img->image.copyTo(*_images[map_idx]);
  _maps_headers[map_idx] = img_msg->header;
  _maps_updates[map_idx] = true;
  // std::cout<<"Frame id: "<< _maps_headers[map_idx].frame_id<<std::endl;
  try {
    _tf_listener.lookupTransform("moving_axis", _maps_headers[map_idx].frame_id,
                                 img_msg->header.stamp, transform);
    _transformations_vector[map_idx] = transform;
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }
  warpMap(map_idx);
}

/**
 * @brief Function that shows the warped maps
 *
 * @param int map_idx
 */

void combineMaps::warpMap(int map_idx) {
  cv::Mat map_warped;
  cv::Mat perspectiveMatrix;

  tf::Quaternion q;
  tf::Matrix3x3 rotation_matrix;
  Eigen::Matrix3d rotation_matrix_eigen;
  cv::Size map_size(_params.width, _params.height);

  // Are all the maps already updated?
  for (auto is_updated : _maps_updates) {
    if (!is_updated) {
      return;
    }
  }

  auto initial_map1 =
      cv_bridge::CvImage{_maps_headers[0], "mono8", *_images[0]}.toImageMsg();
  auto initial_map2 =
      cv_bridge::CvImage{_maps_headers[1], "mono8", *_images[1]}.toImageMsg();

  _image_publisher_map1.publish(initial_map1);
  _image_publisher_map2.publish(initial_map2);

  q = _transformations_vector[map_idx].getRotation();
  tf::Matrix3x3 inverse_transform(q);
  rotation_matrix = inverse_transform.inverse();
  tf::matrixTFToEigen(rotation_matrix, rotation_matrix_eigen);
  // cv::eigen2cv(rotation_matrix_eigen, perspectiveMatrix);

  // /////////////////////////////////////////////////////////////////////////////////
  // /////////////////////////////////////////////////////////////////////////////////
  // cv::Point2f perspectiveSrc[] = {cv::Point2f(370, 412), cv::Point2f(535, 412),
  //                                 cv::Point2f(88, 700), cv::Point2f(858, 700)};
  // cv::Point2f perspectiveDst[] = {cv::Point2f(226, 0), cv::Point2f(737, 0),
  //                                 cv::Point2f(226, 724), cv::Point2f(737, 724)};
  // perspectiveMatrix =
  //     cv::getPerspectiveTransform(perspectiveSrc, perspectiveDst);
  // /////////////////////////////////////////////////////////////////////////////////
  // /////////////////////////////////////////////////////////////////////////////////

  // cv::warpPerspective(*_images[map_idx], *_warped_images[map_idx],
  //                     perspectiveMatrix, map_size);

  // auto warp_map1 =
  //     cv_bridge::CvImage{_maps_headers[1], "mono8", *_warped_images[0]}
  //         .toImageMsg();
  // auto warp_map2 =
  //     cv_bridge::CvImage{_maps_headers[1], "mono8", *_warped_images[1]}
  //         .toImageMsg();
  // _warp_map1.publish(warp_map1);
  // _warp_map2.publish(warp_map2);

  buildMap();
}

/**
 * @brief Function that combines the waped image maps of n cameras
 *
 *
 */

void combineMaps::buildMap() {

  //  auto buffer_warp_maps = std::make_shared<cv::Mat>( _params.height,
  //  _params.width,CV_8UC1); buffer_warp_maps=_warped_images[0];
  cv::Mat buffer_warp_maps;
  _images[0]->copyTo(buffer_warp_maps);

  for (auto is_updated : _maps_updates) {
    if (!is_updated) {
      return;
    }
  }

  for (auto i = 1; i < _images.size(); i++) {
    cv::addWeighted(buffer_warp_maps, 0.5, *_images[i], 0.5, 0,
                    buffer_warp_maps);
  }
  auto final_map =
      cv_bridge::CvImage{_maps_headers[1], "mono8", buffer_warp_maps}
          .toImageMsg();
  _image_map.publish(final_map);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ros::param::get("/dynamicParameters/kernel_size", _params.kernel_size);

  //Cálculo dos parâmetros:
    auto area_probabilistica_ponderada = cv::sum(buffer_warp_maps)[0] / (float)255;
    auto area_total = cv::countNonZero(buffer_warp_maps);
    float area_common=0;

    for (int x = 0; x < buffer_warp_maps.rows; x++) {
      for (int y = 0; y < buffer_warp_maps.cols; y++) {

        if (buffer_warp_maps.at<uchar>(x, y) == 255) {
          area_common++;
        }
      }
    }

    // Cálculos dos indicadores:
    auto I_1 = area_probabilistica_ponderada / area_total;
    auto I_2 = area_common / area_total;



  //  writeResults2CSVfile(_maps_headers[1].seq,
  //                        area_probabilistica_ponderada, area_common, area_total,
  //                        I_1, I_2, _params.kernel_size);
}

/**
 *  @brief Main function
 *
 *
 *
 *
 */

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "combine_multi_cams_node");
  auto params = combineMaps::params{};

  std::vector<std::string> topic_names;
  std::string topic_names_str;

  ros::param::get("~cols_img_small", params.width);
  ros::param::get("~rows_img_small", params.height);
  ros::param::get("~topics_maps", topic_names_str);
  boost::split(topic_names, topic_names_str, boost::is_any_of(","));
  combineMaps maps_combination(topic_names, params);

  // Listening the tranformation to moving_axis:

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}
