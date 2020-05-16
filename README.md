Detection and Tracking of Moving Objects with 2D LIDAR
========================================
This package aims to provide Detection and Tracking of Moving Objects capabilities to robotic platforms that are equipped with a 2D LIDAR sensor and publish 'sensor_msgs/LaseScan' ROS messages.
Such a scenario would be the one visualized below, in which the black scaled car is equipped with a LIDAR sensor and it needs to track the motion of the red vehicle through the LIDAR measurements.\
![Example experiment](https://github.com/kostaskonkk/datmo/raw/master/images/experiment.gif)

The output of this package is visualized below and it can be observed that it estimates position, velocity, orientation and dimensions of the red vehicle.\
![Example experiment](https://github.com/kostaskonkk/datmo/raw/master/images/output.gif)

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
However, since LIDAR measurements become less dense as the distance from the sensor increases, objects
For this reason, the threshold distance should be adapted in a way that it increses in relation with the range distance.
In this system, this is achieved by using the Adaptive Breakpoint Detector algorithm.
Its operation is visualized in the right side of the above figure and the equation that it uses is given below it.

### Rectangle Fitting and L-shape extraction
In this step, rectangles are fitted onto the extracted clusters are fitted with rectangles.
This is done to increase the tracking accuracy and shape estimation of rectangular objects. 
The algorithm used for rectangle fitting is the Search-Based Rectangle Fitting algorithm developed by Zhang et al., 2017 [2].
![Rectangle Fitting](https://github.com/kostaskonkk/datmo/raw/master/images/rectangle_fitting.png)
After rectangle fitting, L-shapes are extracted from all the rectangles.
L-shapes represent the corner of the rectangle closer to the sensor and its two adjacent sides.
Therefore, every L-shape contains five measurements, the position of the corner point, the orientation of the rectangle (theta) and the length of its sides (L1, L2).


## Tracking

The tracking part of the system is visualized in the following flowchart:
![Visualization of the tracking stage](https://github.com/kostaskonkk/datmo/raw/master/images/flowchart_tracking.png)

### Data Association
The clusters are tracked between time frames by a Nearest Neighbour data association scheme, with a Euclidean distance criterion.\
![Visualization of the association algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/association.gif)
<!--![Visualization of the association algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/data_association.gif)-->

### Apperance Change Detector
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
cd ..
catkin_make
```
The datmo package should be now installed to your computer and you will be able to use it after sourcing your workspace. 

```
source devel/setup.bash
```

You can run a demonstration of the DATMO package by running:

```
roslaunch datmo example.launch bag:=overtakes
```

You can run it by typing:

```
roslaunch datmo datmo.launch
```
# ROS API
## Subscribed Topics
scan(sensor_msgs/LaserScan) - This topic should be created be your LIDAR sensor.

## Published Topics

This node can publish a variety of topics but the final configuration depends on the user. By default the majority of the topics are disabled and they should be enabled through the launch file configuration.

**datmo/marker_array (visualization_msgs/MarkerArray)** - In this topic a variety of Rviz markers are published, which can facilitate in understanding the inner workings of the program.\
**datmo/box_kf (datmo/TrackArray)** - In this topic the output of a Kalman Filter with a Constant Velocity model, which tracks the center of the box that surrounds the clusters is published.\

Note: In case that the marker_array topic is published from a robot and visualized in computer, which has a different version of ROS installed (kinetic, melodic, ...), the msgs will not be published and the datmo node will crash with an md5sum error. To mitigate this, you should install on your robot the visualization msgs package of the ROS installation that runs on your computer.

## Custom Messages

This package uses two custom msgs types `datmo/Track` and `datmo/TrackArray` to facilitate the publishing of its results. To my knowledge, at the time of developement, there was no standard ROS messages that accomplishes the same task. 

The `datmo/Track` message has the following structure:\
int32 id - object ID, so it is possible to differentiate between different objects during tracking\
float32 length - Estimated length of the object\
float32 width    Estimated width of the object\
nav_msgs/Odometry odom - Estimated pose of the object

The `datmo/TrackArray` message is an array that contains multiple datmo/Track messages, with the goal of efficient publishing.

## Rviz markers
In case that the  **pub_markers** flag is set to true, this package publishes visualization messages, which can be displayed in Rviz. The following messages are published:

**closest_corner** - The closest corner point of surrounding vehicles is visualized with a black rectangle.\
**bounding_box_center** - The center of the bounding box is visualized with a yellow rectangle.\
**velocities** - The velocities of the tracked objects are represented with an arrow.\



## Parameters

* "lidar_frame" ("string", default: "laser") - Name of the transformation frame (frame_id) of the LaserScan msgs
* "world_frame" ("string", default: "world") - Name of the world coordinate frame, if it is not available, this value can be set equal to the lidar_frame
* "threshold_distance" ("double", default: "0.2") - This value sets the distance that is used by the clustering algorithm
* "euclidean_distance" ("double", default: "0.25") - This value sets the distance that is used by the euclidean distasnce data association algorithm
* "pub_markers" ("bool", default: "false") - publish of the the vizualization markers

# References

[1] D. Kim, K. Jo, M. Lee, and M. Sunwoo, “L-shape model switching-based precise motion tracking of moving vehicles using laser scanners,” IEEE Transactions on Intelligent Transportation Systems, vol. 19, no. 2, pp. 598–612, 2018.\
[2] X. Zhang, W. Xu, C. Dong, and J. M. Dolan, “Efficient l-shape fitting for vehicle detection using laser scanners,” in 2017 IEEE Intelligent Vehicles Symposium (IV), pp. 54–59, IEEE, 2017.

