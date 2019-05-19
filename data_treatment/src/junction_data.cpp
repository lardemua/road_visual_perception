// #include <initializer_list>
#include <vector>
#include <memory>
#include <functional>
#include <mutex>

#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>

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
#include "std_msgs/Header.h"

/*ROS*/
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

struct image_info
{
    cv::Mat image;
    bool isupdate;
    double image_delay;
};

class junction_data
{
public:
    struct params
    {
        int width;
        int height;
        int kernel_size;
    };
    /*Important variables*/
    double alpha_x, pace, scale_factor, cols_img_big, cols_img_small;

    junction_data(const std::vector<std::string> &image_topics, params params);

    void img_callback(const sensor_msgs::ImageConstPtr &img1, int alg_idx);
    void process(int alg_idx);
    void probabilitiesMapImage(cv::Mat &img_intersection, cv::Mat &img_nonintersection);

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

junction_data::junction_data(const std::vector<std::string> &image_topics, params params)
    // clang-format off
    : _params(params)
    , _nh()
    , _it(_nh)
    , _image_publisher_summed(_nh.advertise<sensor_msgs::Image>("draw_prob_map/summed_img", 10))
    , _image_publisher_diff(_nh.advertise<sensor_msgs::Image>("draw_prob_map/intersect_img", 10))
    , _image_publisher_nonint(_nh.advertise<sensor_msgs::Image>("draw_prob_map/nonintersect_img", 10))
    , _prob_map_image(_nh.advertise<sensor_msgs::Image>("draw_prob_map/image_map", 10))
    , _grid_road_map_pub(_nh.advertise<nav_msgs::OccupancyGrid>("draw_prob_map/road_probabilistic_map", 10, true))
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
    CamInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/camera_info", _nh /*, ros::Duration(10)*/);
    k_matrix = CamInfo->K;
    alpha_x = k_matrix[0];
    pace = (1 / (alpha_x)) * (cols_img_big / cols_img_small); // cada lado da celula correponde a este valor em m

    for (auto topic = image_topics.begin(); topic != image_topics.end(); topic++)
    {
        auto topic_idx = std::distance(image_topics.begin(), topic);

        auto callback = boost::bind(&junction_data::img_callback, this, _1, topic_idx);

        auto subscriber = _it.subscribe(*topic, 10, callback);

        _image_subscribers.push_back(subscriber);
    }

    _images.reserve(image_topics.size());
    for (int i = 0; i < image_topics.size(); i++)
    {
        auto img = std::make_shared<cv::Mat>(_params.width, _params.height, CV_8UC3);
        _images.push_back(img);
    }
}

/**
 * @brief Function that receives each image 
 * 
 * @param img_msg image messase
 * @param alg_idx algorithm index 
 */

void junction_data::img_callback(const sensor_msgs::ImageConstPtr &img_msg, int alg_idx)
{
    auto img = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    image_info indiv_img_data; // each image data
    //auto buffer = _images[alg_idx];
    img->image.copyTo(indiv_img_data.image);

    _image_updates[alg_idx] = true;
    indiv_img_data.image_delay=true;
    _images_headers[alg_idx] = img_msg->header;

    t_img = img_msg->header.stamp;
    duration_image = ros::Time::now() - t_img;
    _images_durations.push_back(duration_image.toSec());
    indiv_img_data.image_delay=_images_durations.back();
    ROS_INFO_STREAM("Time to process: " << duration_image.toSec());

    ROS_INFO("received image from algorithm %d", alg_idx);
    images_data_vect.push_back(indiv_img_data);
    process(alg_idx);
}

/**
 * @brief Intersection and disjunction of the images  
 * 
 */

void junction_data::process(int alg_idx)
{
    cv::Mat image_int;
    cv::Mat image_ref;
    cv::Mat image_nonint;

    double delta_timer;
    double time_seconds;
// verificar se as 3 images estão set

    for (auto is_updated : _image_updates)
    {
        if (!is_updated)
        {
            return;
        }
    }

    auto buffer_sum = images_data_vect.at(0).image;
    image_int = images_data_vect.at(0).image;

    cv::cvtColor(image_int, image_int, CV_BGR2GRAY);
    cv::threshold(image_int, image_int, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    for (int i = 1; i < images_data_vect.size(); i++)
    {
        //Too many time of processing? Don't count to the probabilistic map
        if (images_data_vect.at(i).image_delay > 0.6)
        {
            images_data_vect.erase(images_data_vect.begin()+i);
            continue;
        }
        cv::add(buffer_sum, images_data_vect.at(i).image, buffer_sum);
        image_ref = images_data_vect.at(i).image;
        cv::cvtColor(image_ref, image_ref, CV_BGR2GRAY);
        cv::threshold(image_ref, image_ref, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
        cv::bitwise_and(image_int, image_ref, image_int);
        cv::bitwise_xor(image_int, image_ref, image_nonint);
    }

    auto img_msg = cv_bridge::CvImage{_images_headers[0], "bgr8", buffer_sum}.toImageMsg();
    auto img_msg_diff = cv_bridge::CvImage{_images_headers[0], "mono8", image_int}.toImageMsg();
    auto img_msg_nonint = cv_bridge::CvImage{_images_headers[0], "mono8", image_nonint}.toImageMsg();


    _image_publisher_summed.publish(img_msg);
    _image_publisher_diff.publish(img_msg_diff);
    _image_publisher_nonint.publish(img_msg_nonint);

    probabilitiesMapImage(image_int, image_nonint);

    std::fill(_image_updates.begin(), _image_updates.end(), false);
    std::vector<double>().swap(_images_durations);    //Delete and deallocation of the vector _images_durations
    std::vector<image_info>().swap(images_data_vect); //Delete and deallocation of the vector of structures _images_data_vect
}

/**
   * @brief Function tha builds the probability map on an image type.
   *
   * @param mage_intersection intersection zone
   * @param image_nonintersection non intersected zone
   */

void junction_data::probabilitiesMapImage(cv::Mat &image_intersection, cv::Mat &image_nonintersection)
{

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
    int value_filter = 0;
    float prob = 0.0;
    float prob_non_intersect = 0.0;
    int thresh_non_intersect = 1; // valore acrescentado para termos a probabilidade das secções que nao se intersectam

    image_intersection.convertTo(image_intersection, CV_32F);
    image_nonintersection.convertTo(image_nonintersection, CV_32F);
    image_nonintersection = image_nonintersection / (float)(_params.kernel_size + thresh_non_intersect);

    kernel = cv::Mat::ones(_params.kernel_size, _params.kernel_size, CV_32F) / (float)(_params.kernel_size * _params.kernel_size);
    /// Initialize arguments for the filter
    anchor = cv::Point(-1, -1);
    delta = 0;
    ddepth = -1;

    cv::filter2D(image_intersection, img_filt, ddepth, kernel, anchor, delta, cv::BORDER_DEFAULT);
    img_final = cv::Mat::zeros(image_intersection.rows, image_intersection.cols, CV_8UC1);
    img_filt = img_filt + image_nonintersection;

    //Probabilities calculations:

    // for (x = 0; x < image_intersection.rows; x++)
    // {
    //     for (y = 0; y < image_intersection.cols; y++)
    //     {
    //         value_filter = img_filt.at<uchar>(x, y);
    //         prob = value_filter / (float)255;
    //         pix_cinzentosf = prob * (float)255;
    //         img_final.at<uchar>(x, y) = (int)pix_cinzentosf;
    //     }
    // }

    //Image result:-----------------------------------------------------
    //   if (current_image_original)
    //   {
    //     img_original = current_image_original->image;
    //     resize(img_original, img_original, size);
    //     cvtColor(img_filt, img_filt, CV_GRAY2BGR);
    //     img_filt.convertTo(img_filt, CV_8UC3);
    //     img_original.convertTo(img_original, CV_8UC3);
    //     addWeighted(img_filt, 0.5, img_original, 1, 0, final_result);
    //     image_final_result = cv_bridge::CvImage{current_image_alg1->header, "bgr8", final_result}.toImageMsg();
    //   }
    //-------------------------------------------------------------------
    // cv::cvtColor(img_filt, img_filt, CV_BGR2GRAY);
    img_filt.convertTo(img_filt, CV_8UC1);

    //GridMap:-----------------------------------------------------
    auto img_final_map = cv_bridge::CvImage{_images_headers[0], "mono8", img_filt}.toImageMsg();
    _prob_map_image.publish(img_final_map);

    info.height = image_intersection.cols;
    info.width = image_intersection.rows;
    info.resolution = pace;
    info.origin.position.x = 0;
    info.origin.position.y = -(info.height * pace / 2);
    info.origin.position.z = 0;
    grid_map::GridMapRosConverter::initializeFromImage(*img_final_map, pace, grid_road_GridMap);
    grid_map::GridMapRosConverter::addLayerFromImage(*img_final_map, "road probability map", grid_road_GridMap, 0, 255, 128);
    grid_map::GridMapRosConverter::toOccupancyGrid(grid_road_GridMap, "road probability map", 0, 255, roadmapGrid);
    roadmapGrid.info = info;
    roadmapGrid.header = _images_headers[0];
    roadmapGrid.header.frame_id = "/world";
    _grid_road_map_pub.publish(roadmapGrid);
    // cout << "Header: " << roadmapGrid.header << endl;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "draw_prob_map_node");
    auto params = junction_data::params{};
    ros::param::get("~cols_img_small", params.width);
    ros::param::get("~rows_img_small", params.height);
    ros::param::get("~kernel_size", params.kernel_size);
    std::vector<std::string> topic_names;
    std::string topic_names_str;
    ros::param::get("~topics_polygons", topic_names_str);
    boost::split(topic_names, topic_names_str, boost::is_any_of(","));
    auto data_merger = junction_data(topic_names, params);

    ros::spin();

    return 0;
}
