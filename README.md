# Visual Perception of the road onboard the AtlasCar

This is my Mechanical Engineering Masters Thesis Repo in the fielf of Artificial Vision applied to Autonomous Driving.

## Where and when was it made?

Department of Mechanical Engineering (DEM), University of Aveiro (UA)

LAR: Laboratory of Automation and Robotics

2019

### Advisor

Profesor Vitor Santos

DEM, UA

Aveiro, Portugal

### Prerequisites

What things that are needed:

* [ROS-melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) - The ROS version used.
* [FlyCapture 2.x](https://www.ptgrey.com/support/downloads)
* [Camera Pointgrey_driver](https://github.com/ros-drivers/pointgrey_camera_driver) - The driver used for the camera. 

Hardware:
* [Camera PointGrey FL3-GE-28S4-C](https://www.ptgrey.com/support/downloads/10119/) - The camera used


### Project Guide

After all of the prerequisites are installed, the first step is to calibrate the camera in order to get the intrinsics parameters. This was based in this [Tutorial](http://wiki.ros.org/camera_calibration).

```
To run the camera_calibrator package: rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw
```

To get the undistorted image (this step is already included on the my camera_driver launch file):

```
$ ROS_NAMESPACE=my_camera rosrun image_proc image_proc
```
In order to get faster results, I rescale the initial image inside of my architecture. At this moment, to change the resolution it is necessary follow the steps below:
1) Accordingly,comment/uncomment the perspectiveSrc and the pespectiveDst variables in advanced_lane_detection/src/main.cpp because they respresent the perspective transformation (IPM);
2) In advanced_lane_detection/src/laneDetection.cpp (lines 383/384) ucomment or comment the lines to show the right result;
3) In Lane_Detector/params/Detector2.yaml change the value's variable ipmRight to number of columns of the image;
4) In Lane_Detector/cfg/Detector.cfg (lines 47-70) uncomment or comment the lines based on the image resolution;
5) On the launch files:
    -camera_tiago.launch;
    -data_treatment.launch;
    -advaced_lane_detection.launch;
    Change the parmaters related to the image resolution.


## Usage

To launch everything:

```
roslaunch pointgrey_camera_driver camera_tiago.launch packet_resend:=false
```

### Algorithms that were used

This project uses algorithms from different authors:
 * [Nsteel/LaneDetection](https://github.com/Nsteel/Lane_Detector)
 * [HsucheChiang/Advanced_Lane_Detection](https://github.com/HsucheChiang/Advanced_Lane_Detection)


## Author

* **Tiago Almeida** - [My Thesis Blog](https://tmralmeida.github.io/thesis_blog/)

