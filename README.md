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
To run the camera_calibrator package: rosrun camera_calibration cameracalibrator.py --size 8*6 --square 0.108 image:=/camera/image_raw
```

To get the undistorted image (this step is already included on the my camera_driver launch file):

```
$ ROS_NAMESPACE=my_camera rosrun image_proc image_proc
```



## Usage

To launch everything:

```
roslaunch pointgrey_camera_driver camera.launch packet_resend:=false
```

### Algorithms that were used

In this project were used some algorithms:
 * [Nsteel/LaneDetection](https://github.com/Nsteel/Lane_Detector)


## Author

* **Tiago Almeida** - [My Thesis Blog](https://tmralmeida.github.io/thesis_blog/)

