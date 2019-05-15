#include "ros/ros.h"
#include <math.h>       /* atan */
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8.h"
#include <visualization_msgs/Marker.h>
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
// transform conversion
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_broadcaster.h>




typedef std::pair<double, double> Point;
typedef std::pair<double, double> coordinates;
typedef std::vector<Point> pointList;




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
    marker_pub = n.advertise<visualization_msgs::Marker>("markers", 1000);// if it is smaller some things are not published properly
    vehicles_pub = n.advertise<geometry_msgs::PoseArray>("vehicles", 100);
    sub_scan = n.subscribe("/scan", 1, &datmo::callback, this);

  }

  void callback(const sensor_msgs::LaserScan::ConstPtr &);
  void Clustering(const sensor_msgs::LaserScan::ConstPtr& , vector<pointList> &);
  void pubPosesArrayVehicles();



private:

  ros::NodeHandle n; 
  ros::Publisher marker_pub;
  ros::Publisher vehicles_pub;

  //Tuning Parameteres
  unsigned int tp_dth = 63;


  int time;

  ros::Subscriber sub_scan;
  sensor_msgs::LaserScan scan;
  vector<Cluster> clusters;



  unsigned long int cl       = 0;//counter for the l_shape to be used as id for the markers
  unsigned long int cg       = 0;//group counter to be used as id for the markers
  unsigned long int cclusters= 0;//counter for the cluster objects to be used as id for the markers


};
