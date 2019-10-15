Detection and Tracking of Moving Objects with 2D LIDAR
========================================
# Overview 
Detection and Tracking of Moving Objects using `sensor_msgs/LaserScan`. This node can be used to detect and track objects or it can be used solely for its data clustering, data association and rectangle fitting functions. The workflow of this package is inspired by the one presented in Kim et al., 2018 [1]. 

* Clustering - this node clusters data points by using the Adaptive Breakpoint Detector algorithm.\
![Visualization of the clustering algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/clustering.gif)
* Data Association - the clusters are tracked between time frames by a Nearest Neighbour data association scheme, with a Mahalanobis Distance criterion.\
![Visualization of the association algorithm](https://github.com/kostaskonkk/datmo/raw/master/images/data_association.gif)
* Rectangle Fitting - the clusters are furthermore fitted with rectangles to facilitate the tracking and shape estimation of vehicles. The rectangle fitting is based on the Search-Based Rectangle Fitting algorithm developed by Zhang et al., 2017 [2].

In this [video](https://youtu.be/HfFZcYwsY3I?t=646) a presentation can be found, in which I explain some early features of this package.

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

# ROS API
## Subscribed Topics
scan(sensor_msgs/LaserScan) - This topic should be created be your LIDAR sensor.

## Published Topics

This node can publish a variety of topics but the final configuration depends on the user. By default the majority of the topics are disabled and they should be enabled through the launch file configuration.

**marker_array (visualization_msgs/MarkerArray)** - In this topic a  variety of Rviz markers are published, which can facilitate in understanding the inner workings of the program.\
**mean_tracks (datmo/TrackArray)** - In this topic the mean coordinates of each cluster is published.\
**filtered_tracks (datmo/TrackArray)** - In this topic the output of a Kalman Filter with a Constant Velocity model, which tracks the clusters are published.\
**box_tracks (datmo/TrackArray)** - In this topic the  output of a Kalman Filter with a Constant Velocity model, which tracks the center of the box that surrounds the clusters is published.

Note: In case that the marker_array topic is published from a robot and visualized in computer, which has a different version of ROS installed (kinetic, melodic, ...), the msgs will not be published and the datmo node will crash with an md5sum error. To mitigate this, you should install on your robot the visualization msgs package of the ROS installation that runs on your computer.

## Custom Messages

This package uses two custom msgs types `datmo/Track` and `datmo/TrackArray` to facilitate the publishing of its results. To my knowledge, at the time of developemnt, there was no standard ROS messages that accomplishes the same task. 

The `datmo/Track` message has the following structure:\
int32 id - object ID, so it is possible to differentiate between different objects during tracking\
float32 length - Estimated length of the object\
float32 width    Estimated width of the object\
nav_msgs/Odometry odom - Estimated pose of the object

The `datmo/TrackArray` message is just a vector that can contain multiple datmo/Track messages, with the goal of efficient publishing.

## Parameters

* "lidar_frame" ("string", default: "laser") - Name of the transformation frame (frame_id) of the LaserScan msgs
* "world_frame" ("string", default: "world") - Name of the world coordinate frame, if it is not available, this value can be set equal to the lidar_frame
* "threshold_distance" ("double", default: "0.2") - This value sets the distance that is used by the clustering algorithm
* "euclidean_distance" ("double", default: "0.25") - This value sets the distance that is used by the euclidean distasnce data association algorithm
* "pub_markers" ("bool", default: "false") - publish of the the vizualization markers

# TODO

- [ ] Implement an Unscented Kalman Filter
- [ ] Improve the shape estimation and yaw estimation of objects

# References

[1] D. Kim, K. Jo, M. Lee, and M. Sunwoo, “L-shape model switching-based precise motion tracking of moving vehicles using laser scanners,” IEEE Transactions on Intelligent Transportation Systems, vol. 19, no. 2, pp. 598–612, 2018.\
[2] X. Zhang, W. Xu, C. Dong, and J. M. Dolan, “Efficient l-shape fitting for vehicle detection using laser scanners,” in 2017 IEEE Intelligent Vehicles Symposium (IV), pp. 54–59, IEEE, 2017.


