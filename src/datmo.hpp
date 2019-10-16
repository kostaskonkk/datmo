#include <ros/ros.h>
#include <math.h>       /* atan */
#include <omp.h>      //Multi-threading
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()

#include <chrono>
#include <iostream>
#include <fstream>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/tf.h>
#include <datmo/TrackArray.h>
#include <datmo/Track.h>

#include "cluster.hpp"

typedef std::pair<double, double> Point;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;


using namespace std;
// This node segments the point cloud based on the break-point detector algorithm.
// This algorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking 
// of Moving Vehicles Using Laser Scanners.
class Datmo
{
public:
  Datmo();
  ~Datmo();

  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  void visualiseGroupedPoints(const vector<pointList> &);
  void transformPointList(const pointList& , pointList& );

  tf::TransformListener tf_listener;
private:
  ros::Publisher pub_marker_array; 
  ros::Publisher pub_tracks_mean;
  ros::Publisher pub_tracks_mean_kf;
  ros::Publisher pub_tracks_box;

  ros::Subscriber sub_scan;
  sensor_msgs::LaserScan scan;
  vector<Cluster> clusters;

  ofstream whole; // file to write the program duration
  ofstream clustering; // file to write the program duration
  ofstream rect_fitting; //write rectangle fitting duration
  ofstream testing; //various testing

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
  bool w_exec_times;
  string lidar_frame;
  string world_frame;

};
