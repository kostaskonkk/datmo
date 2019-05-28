#include "ros/ros.h"
#include <math.h>       /* atan */
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()
//Douglas Peucker algorithm
#include <iostream>
#include <cmath>
#include <utility>
#include <stdexcept>
//
#include "cluster.h"

#include <Eigen/Dense>
// #include <jsk_recognition_msgs/BoundingBox.h>//Only used for bounding boxes
// transform conversion
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


#include "tf/transform_listener.h"
#include <tf/transform_broadcaster.h>

// #include <midi_ros/midi.h> //parameter tuning through midi controller



typedef std::pair<double, double> Point;
typedef std::pair<double, double> coordinates;
typedef std::vector<double> l_shape;
typedef std::vector<l_shape> l_shapes;
typedef std::vector<Point> pointList;

// typedef std::vector<pointList*> pointer_clusters;



using namespace std;
using namespace Eigen;
// This node segments the point cloud based on the break-point detector algorithm.
// This alogorithm is based on "L-Shape Model Switching-Based Precise Motion Tracking 
// of Moving Vehicles Using Laser Scanners.
class datmo
{
public:
  datmo()
  {
    ros::NodeHandle n; 
    ros::NodeHandle n_private("~");
    ROS_INFO("Starting Detection And Tracking of Moving Objects");

    n_private.param("pub_markers", p_marker_pub, false);
    n_private.param("pub_vehicles_InBox", p_vehicles_InBox_pub, false);
    n_private.param("pub_vehicles_pub", p_vehicles_pub, false);
    n_private.param("pub_vel_vehicles_pub", p_vel_vehicles_pub, false);
    n_private.param("pub_odom_pub", p_odom_pub, false);
    n_private.param("pub_odom_filtered_pub", p_odom_filtered_pub, false);
    n_private.param("pub_trajectories", p_trajectories_pub, false);
 

    marker_pub = n.advertise<visualization_msgs::Marker>("markers", 1000);// if it is smaller some things are not published properly
    marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 1000);
    trajectory_pub = n.advertise<nav_msgs::Path>("trajectories", 1000);
    // bb_pub = n.advertise<jsk_recognition_msgs::BoundingBox>("bb", 20);
    vehicles_pub = n.advertise<geometry_msgs::PoseArray>("vehicles", 100);
    vehicles_InBox_pub = n.advertise<geometry_msgs::PoseArray>("vehicles_inBox", 100);
    vel_vehicles_pub = n.advertise<geometry_msgs::PoseArray>("vel_vehicles", 100);
    sub_scan = n.subscribe("/scan", 1, &datmo::callback, this);
    // sub_pose = n.subscribe("/mocap_pose", 1, &datmo::tf_callback, this);
    // sub_midi = n.subscribe("/midi", 1, &datmo::midi_callback, this);
    // L_pub = n.advertise<geometry_msgs::Point>("l_shapes", 1000);

    odom_pub = n.advertise<nav_msgs::Odometry>("odom_objects", 1000);
    odom_filtered_pub = n.advertise<nav_msgs::Odometry>("odom_filtered_objects", 1000);
  }

  // void midi_callback(const std_msgs::Int8::ConstPtr &);
  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void l_shape_extractor(const vector<Point> &, vector<double> &, bool);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);

  void pubPosesArrayVehicles();
  void pubPosesArrayVehiclesInsideBox(double halfwidth);
  void pubTrajectories();
  void pubVelArrayVehicles();

  void pubOdomObjects();
  void pubFilteredOdomObjects();

  tf::TransformListener tf_;
private:
  ros::Publisher trajectory_pub;  
  ros::Publisher seg_pub_1;
  ros::Publisher marker_pub;
  ros::Publisher marker_array_pub; 
  ros::Publisher vehicles_InBox_pub;
  // ros::Publisher bb_pub;
  ros::Publisher vehicles_pub;
  ros::Publisher vel_vehicles_pub;

  ros::Publisher odom_pub;
  ros::Publisher odom_filtered_pub;

  // ros::Publisher L_pub;
  tf::TransformBroadcaster tf_br;
  tf::Transform tf_world_base_link;
  tf::Transformer transformer;

  ros::Subscriber sub_midi;
  ros::Subscriber sub_scan;
  ros::Subscriber sub_pose;
  sensor_msgs::LaserScan scan;

  // vector<KalmanFilter> filters;
  vector<Cluster> clusters;


  //Tuning Parameteres
  unsigned int tp_dth = 63;
  unsigned int time;
  double t, dt;

  unsigned long int cfilters = 0;//counter for the filters to be used as id for the markers
  unsigned long int cl       = 0;//counter for the l_shape to be used as id for the markers
  unsigned long int cg       = 0;//group counter to be used as id for the markers
  unsigned long int cclusters= 0;//counter for the cluster objects to be used as id for the markers


  //Parameters
  bool p_marker_pub;
  bool p_vehicles_InBox_pub;
  bool p_vehicles_pub;
  bool p_vel_vehicles_pub;
  bool p_odom_pub;
  bool p_odom_filtered_pub;
  bool p_trajectories_pub;

};
