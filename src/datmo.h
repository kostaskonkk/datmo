#include <ros/ros.h>
#include <math.h>       /* atan */
#include <vector>
#include <random>
#include <algorithm> // for sort(), min()
//Douglas Peucker algorithm
#include <iostream>
#include <cmath>
#include <utility>
#include <stdexcept>
//

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <datmo/TrackArray.h>
#include <datmo/Track.h>

#include "cluster.h"
// #include <midi_ros/midi.h> //parameter tuning through midi controller



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
  Datmo()
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
 

    tracks_pub = n.advertise<datmo::TrackArray>("tracks", 1);
    filtered_tracks_pub = n.advertise<datmo::TrackArray>("filtered_tracks", 1);
    marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 10);
    trajectory_pub = n.advertise<nav_msgs::Path>("trajectories", 1000);
    //vehicles_pub = n.advertise<geometry_msgs::PoseArray>("vehicles", 100);
    debug_pub = n.advertise<geometry_msgs::Quaternion>("debug", 100);
    //vel_vehicles_pub = n.advertise<geometry_msgs::PoseArray>("vel_vehicles", 100);
    sub_scan = n.subscribe("/scan", 1, &Datmo::callback, this);
    // sub_pose = n.subscribe("/mocap_pose", 1, &Datmo::tf_callback, this);
    // sub_midi = n.subscribe("/midi", 1, &datmo::midi_callback, this);

  }

  // void midi_callback(const std_msgs::Int8::ConstPtr &);
  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  void visualiseGroupedPoints(const vector<pointList> &);
  //void pubPosesArrayVehicles();
  //void pubPosesArrayVehiclesInsideBox(double halfwidth);
  void pubTrajectories();
  //void pubVelArrayVehicles();

  tf::TransformListener tf_;
private:
  ros::Publisher trajectory_pub;  
  //ros::Publisher seg_pub_1;
  ros::Publisher marker_array_pub; 
  ros::Publisher debug_pub;
  //ros::Publisher vehicles_pub;
  //ros::Publisher vel_vehicles_pub;
  ros::Publisher tracks_pub;
  ros::Publisher filtered_tracks_pub;

  tf::TransformBroadcaster tf_br;
  tf::Transform tf_world_base_link;
  tf::Transformer transformer;

  //ros::Subscriber sub_midi;
  ros::Subscriber sub_scan;
  //ros::Subscriber sub_pose;
  sensor_msgs::LaserScan scan;

  vector<Cluster> clusters;

  //Tuning Parameteres
  unsigned int tp_dth = 63;
  double dt;
  ros::Time time;

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
