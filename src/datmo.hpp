#include <ros/ros.h>
#include <math.h>       /* atan */
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()

#include <chrono>
#include <iostream>
#include <fstream>

//Douglas Peucker algorithm
//#include <iostream>
//#include <cmath>
//#include <utility>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
//#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <datmo/TrackArray.h>
#include <datmo/Track.h>

#include "cluster.hpp"



typedef std::pair<double, double> Point;
typedef std::pair<double, double> coordinates;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;


using namespace std;
// This node segments the point cloud based on the break-point detector algorithm.
// This alogorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking 
// of Moving Vehicles Using Laser Scanners.
class Datmo
{
public:
  Datmo();
  ~Datmo();

  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  void visualiseGroupedPoints(const vector<pointList> &);
  void pubTrajectories();

  tf::TransformListener tf_;
private:
  ros::Publisher trajectory_pub;  
  ros::Publisher marker_array_pub; 
  ros::Publisher debug_pub;
  ros::Publisher tracks_pub;
  ros::Publisher filtered_tracks_pub;
  ros::Publisher box_tracks_pub;

  //tf::TransformBroadcaster tf_br;
  //tf::Transform tf_world_base_link;
  //tf::Listener tf_listener;
  tf::Transformer tf;

  ros::Subscriber sub_scan;
  sensor_msgs::LaserScan scan;
  vector<Cluster> clusters;

  ofstream whole; // file to write the program duration
  ofstream clustering; // file to write the program duration
  ofstream rect_fitting; //write rectangle fitting duration

  //Tuning Parameteres
  double dt;
  ros::Time time;

  unsigned long int cg       = 1;//group counter to be used as id of the clusters
  //initialised as one, because 0 index take the msgs that fail to be initialized
  unsigned long int cclusters= 0;//counter for the cluster objects to be used as id for the markers


  //Parameters
  double dth;
  double euclidean_distance;
  bool p_marker_pub;
  bool p_vehicles_InBox_pub;
  bool p_vehicles_pub;
  bool p_vel_vehicles_pub;
  bool p_odom_pub;
  bool p_odom_filtered_pub;
  bool p_trajectories_pub;
  bool w_exec_times;
  string lidar_frame;
  string world_frame;

};
