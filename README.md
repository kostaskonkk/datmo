Detection and Tracking of Moving Objects with 2D LIDAR
========================================
This package aims to provide Detection and Tracking of Moving Objects capabilities to robotic platforms that are equipped with a 2D LIDAR sensor and publish 'sensor_msgs/LaseScan' ROS messages.
It specializes in tracking of rectangle shaped objects and therefore it is most used in vehicle tracking.
The workflow of this package is inspired by the one presented in Kim et al., 2018 [1]. 

# Overview 
Below you can read a synopsis of its funcion and operation. 
A more in depth explanation of this package's inner workings is given in this [paper](https://github.com/kostaskonkk/datmo/raw/master/paper.pdf).

## Detection 

The detection part of the system is visualized in the following flowchart:
![Visualization of the detection stage](https://github.com/kostaskonkk/datmo/raw/master/images/flowchart_detection.png)

### Clustering 
In the clustering step the raw LIDAR measurements are divided to groups/clusters. In this way, the different objects in the environment are differentiatted.
A simple way to do this is by separating clusters, based on the inbetween euclidean distance of LIDAR measurements. 
Therefore, if the distance of two consequtive LIDAR measurements is greater than a predefined threshold distance the two points are divided in two separate clusters.
![Visualization of the breakpoint clustering algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/clustering.png)
The LIDAR measurements are clustered with the Adaptive Breakpoint Detector algorithm.

### Rectangle Fitting and L-shape extraction
The clusters are furthermore fitted with rectangles to facilitate the tracking and shape estimation of vehicles. 
The rectangle fitting is based on the Search-Based Rectangle Fitting algorithm developed by Zhang et al., 2017 [2].
![Rectangle Fitting](https://github.com/kostaskonkk/datmo/raw/master/images/rectangle_fitting.png)


## Tracking

The tracking part of the system is visualized in the following flowchart:
![Visualization of the tracking stage](https://github.com/kostaskonkk/datmo/raw/master/images/flowchart_tracking.png)

### Data Association
The clusters are tracked between time frames by a Nearest Neighbour data association scheme, with a Mahalanobis Distance criterion.\
![Visualization of the association algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/association.gif)
<!--![Visualization of the association algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/data_association.gif)-->

### Corner Switching
In cases that the closest corner point of a tracked vehicle changes between measurements, this is detected by comparing the Mahalanobis distance of the four corner points of the vehicle with that of the new L-shape.
![Visualization of the corner switching scheme](https://github.com/kostaskonkk/datmo/raw/master/images/corner.gif)
<!--![Visualization of the association algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/data_association.gif)-->


### Kinematic and Shape Trackers

The motion of the detected vehicles is tracked based on two kinematic trackers.
A Kalman Filter with a Constant Velocity Model and an Unscented Kalman Filter with a Coordinated-Turn model.
![Kinematic Trackers](https://github.com/kostaskonkk/datmo/raw/master/images/kinematic.png)

The shape and orientation of the detected vehicles are tracked by a Kalman Filter that contains two models.
The first model is a Constant Shape model and it indicates that the shape of the detected vehicle remains constant.
The second model is a Constant Turn Rate model that indicates that the turn rate of the detected vehicle remains constant, while its orientation depends on the turn rate.
![Kinematic Trackers](https://github.com/kostaskonkk/datmo/raw/master/images/shape.png)

Below you can find a video of a presentation of mine, in which I explain some early features of this package.

[![Midterm presentation](https://img.youtube.com/vi/HfFZcYwsY3I/0.jpg)](https://www.youtube.com/watch?v=HfFZcYwsY3I "Midterm presentation")

# Installation and use
This ROS package can be installed in the following way:
1. First you should navigate to the source folder of your catkin_ws. For example `cd ~/catkin_ws/src`.
2. Run 
```sh
git clone git@github.com:kostaskonkk/datmo.git
cd ../..
catkin_make
```
The datmo package should be now installed to your computer. You can run it by typing:

```
roslaunch datmo datmo.launch
```
You can run a demo of it by typing:

```
roslaunch datmo example.launch bag:=overtakes
```
# ROS API
## Subscribed Topics
scan(sensor_msgs/LaserScan) - This topic should be created be your LIDAR sensor.

## Published Topics

This node can publish a variety of topics but the final configuration depends on the user. By default the majority of the topics are disabled and they should be enabled through the launch file configuration.

**marker_array (visualization_msgs/MarkerArray)** - In this topic a  variety of Rviz markers are published, which can facilitate in understanding the inner workings of the program.\
**tracks/box_kf (datmo/TrackArray)** - In this topic the  output of a Kalman Filter with a Constant Velocity model, which tracks the center of the box that surrounds the clusters is published.
**tracks/box_ukf (datmo/TrackArray)** - In this topic the output of an Unscented Kalman Filter (UKF) with an omnidirectional motion model, which tracks the center of the box that surrounds the clusters is published.

Note: In case that the marker_array topic is published from a robot and visualized in computer, which has a different version of ROS installed (kinetic, melodic, ...), the msgs will not be published and the datmo node will crash with an md5sum error. To mitigate this, you should install on your robot the visualization msgs package of the ROS installation that runs on your computer.

## Custom Messages

This package uses two custom msgs types `datmo/Track` and `datmo/TrackArray` to facilitate the publishing of its results. To my knowledge, at the time of developemnt, there was no standard ROS messages that accomplishes the same task. 

The `datmo/Track` message has the following structure:\
int32 id - object ID, so it is possible to differentiate between different objects during tracking\
float32 length - Estimated length of the object\
float32 width    Estimated width of the object\
nav_msgs/Odometry odom - Estimated pose of the object

The `datmo/TrackArray` message is an array that contains multiple datmo/Track messages, with the goal of efficient publishing.

## Parameters

* "lidar_frame" ("string", default: "laser") - Name of the transformation frame (frame_id) of the LaserScan msgs
* "world_frame" ("string", default: "world") - Name of the world coordinate frame, if it is not available, this value can be set equal to the lidar_frame
* "threshold_distance" ("double", default: "0.2") - This value sets the distance that is used by the clustering algorithm
* "euclidean_distance" ("double", default: "0.25") - This value sets the distance that is used by the euclidean distasnce data association algorithm
* "pub_markers" ("bool", default: "false") - publish of the the vizualization markers

# References

[1] D. Kim, K. Jo, M. Lee, and M. Sunwoo, “L-shape model switching-based precise motion tracking of moving vehicles using laser scanners,” IEEE Transactions on Intelligent Transportation Systems, vol. 19, no. 2, pp. 598–612, 2018.\
[2] X. Zhang, W. Xu, C. Dong, and J. M. Dolan, “Efficient l-shape fitting for vehicle detection using laser scanners,” in 2017 IEEE Intelligent Vehicles Symposium (IV), pp. 54–59, IEEE, 2017.

