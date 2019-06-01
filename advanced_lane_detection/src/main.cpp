/**
 * @file main.cpp
 * @author Tiago Almeida (tm.almeida@ua.pt)
 * @brief Turn an algorithm a ros package
 * @version 0.1
 * @date 2019-04-17
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

/*ROS*/
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

#include <iostream>
#include <string>
#include "advanced_lane_detection/laneDetection.h"

/*Namesapaces*/
using namespace std;
using namespace ros;
using namespace cv;

//Global Variables
//Img size 964*724:
// Point2f perspectiveSrc[] = {Point2f(225, 362), Point2f(739, 362), Point2f(88, 700), Point2f(858, 700)};
// Point2f perspectiveDst[] = {Point2f(226, 0), Point2f(737, 0), Point2f(226, 724), Point2f(737, 724)};
//Img size 640*480
// Point2f perspectiveSrc[] = {Point2f(125, 262), Point2f(400, 262), Point2f(44, 453), Point2f(550, 453)};
// Point2f perspectiveDst[] = {Point2f(126, 0), Point2f(426, 0), Point2f(126, 480), Point2f(426, 480)};
//Img size (320*240)
// Point2f perspectiveSrc[] = {Point2f(83, 61), Point2f(220, 61), Point2f(42, 300), Point2f(270, 300)};
// Point2f perspectiveDst[] = {Point2f(63, 0), Point2f(263, 0), Point2f(63, 240), Point2f(263, 240)};

//Oboard Atlas
Point2f perspectiveSrc[] = {Point2f(120, 135), Point2f(160, 135), Point2f(30, 220), Point2f(260, 220)};
Point2f perspectiveDst[] = {Point2f(63, 0), Point2f(263, 0), Point2f(63, 240), Point2f(263, 240)};


class alg2
{
  public:
    alg2();
    void Looping();

  private:
    ros::NodeHandle n;

    /*Important Variables*/
    Mat imgPerspective;
    Mat perspectiveMatrix; //Homography Matrix.
    Mat warpEdge;
    Mat imageRedChannel;
    Mat redBinary;
    Mat mergeImage;
    Mat histImage;
    Mat warpMask;
    Mat maskImage;
    Mat finalResult;
    Size frameSize;
    int cols_resize = 964; //default parameters
    int rows_resize = 724; // default parameters

    // bool info_set = false;

    /*ROS*/
    image_transport::ImageTransport it;

    /*Publishers && Subs*/
    ros::Publisher initial_image;
    ros::Publisher poly_image;
    ros::Subscriber camInfo;

    image_transport::Subscriber sub_img;

    /*Images pointers reveive*/
    cv_bridge::CvImagePtr current_image;

    /*Images messages*/
    sensor_msgs::ImagePtr img_init;
    sensor_msgs::ImagePtr poly_draw;

    /*Functions*/
    void Publishers();
    void receiveInitImg(const sensor_msgs::ImageConstPtr &img);
    void processFrames();

    // void callbackCamInfo(const sensor_msgs::CameraInfo::ConstPtr &cm, bool *done);
};

/**
 * @brief Class constructor 
 * 
 */

alg2::alg2() : it(n)
{
    //Resize image
    ros::param::get("~cols_resize", cols_resize);
    ros::param::get("~rows_resize", rows_resize);
    

    /*Publishers*/
    sub_img = it.subscribe("/camera/image_rect_color", 10, &alg2::receiveInitImg, this);
    poly_image = n.advertise<sensor_msgs::Image>("/advanced_algorithm/polygon", 10);
    initial_image = n.advertise<sensor_msgs::Image>("/advanced_algorithm/finalResult", 10);
    // camInfo = n.subscribe<sensor_msgs::CameraInfo>("/camera/camera_info", 10, std::bind(readCameraInfo, std::placeholders::_1, &info_set));
}

/**
 * @brief Loop that runs the main functions
 * 
 */

void alg2::Looping()
{
    if (current_image)
    {
        processFrames();
        Publishers();
    }
}

/**
 * @brief Publishers function
 * 
 * @param i
 */
void alg2::Publishers()
{
    if (current_image)
    {
        poly_image.publish(poly_draw);
        initial_image.publish(img_init);
    }
}

/**
 * @brief Callback that receives the image from the camera already retified 
 * 
 * @param img 
 */
void alg2::receiveInitImg(const sensor_msgs::ImageConstPtr &img)
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
 * @brief process algorithm application
 * 
 */

void alg2::processFrames()
{
    Size size_img(cols_resize, rows_resize);
    Mat init_img = current_image->image;
    resize(init_img, init_img, size_img);
    perspectiveMatrix = getPerspectiveTransform(perspectiveSrc, perspectiveDst);
    

    // draw the roi (for perspective transform)
    // line(init_img, perspectiveSrc[0], perspectiveSrc[1], Scalar(0, 0, 255), 2);
    // line(init_img, perspectiveSrc[1], perspectiveSrc[3], Scalar(0, 0, 255), 2);
    // line(init_img, perspectiveSrc[3], perspectiveSrc[2], Scalar(0, 0, 255), 2);
    // line(init_img, perspectiveSrc[2], perspectiveSrc[0], Scalar(0, 0, 255), 2);
    // circle(init_img, perspectiveSrc[0], 6, Scalar(0, 0, 255), CV_FILLED);
    // circle(init_img, perspectiveSrc[1], 6, Scalar(0, 0, 255), CV_FILLED);
    // circle(init_img, perspectiveSrc[2], 6, Scalar(0, 0, 255), CV_FILLED);
    // circle(init_img, perspectiveSrc[3], 6, Scalar(0, 0, 255), CV_FILLED);
    //frameSize = init_img.size();

    

    
    //Applying lane detection algorithm
    laneDetection LaneAlgo(init_img, perspectiveMatrix);
    LaneAlgo.laneDetctAlgo();

    warpEdge = LaneAlgo.getWarpEdgeDetectResult().clone();
    imageRedChannel = LaneAlgo.getRedChannel().clone();
    redBinary = LaneAlgo.getRedBinary().clone();
    mergeImage = LaneAlgo.getMergeImage().clone();
    histImage = LaneAlgo.getHistImage().clone();
    maskImage = LaneAlgo.getMaskImage().clone();
    warpMask = LaneAlgo.getWarpMask().clone();
    finalResult = LaneAlgo.getFinalResult().clone();

    // init_img.convertTo(init_img,CV_8UC3);

    poly_draw = cv_bridge::CvImage{current_image->header, "bgr8", warpMask}.toImageMsg();
    img_init = cv_bridge::CvImage{current_image->header, "bgr8", finalResult}.toImageMsg();
}

/**
 * @brief Main function
 *
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv,"advanced_algorithm_node" );
    alg2 processImage;

    while (ros::ok())
    {
        processImage.Looping();
        ros::spinOnce();
    }

    return 0;
}