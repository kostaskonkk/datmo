#pragma once
#include <ros/ros.h>
#include "kalman-cpp/kalman.hpp"
#include "l_shape_tracker.hpp"
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "datmo/Track.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;
const double pi = 3.141592653589793238463; 

class Cluster {
public:

  Cluster(unsigned long int id, const pointList&, const double&, const string&, const tf::Transform& ego_pose);


  string frame_name;
  Point ego_coordinates;

  datmo::Track msg_track_mean;
  datmo::Track msg_track_mean_kf;
  datmo::Track msg_track_box;

  unsigned long int id; //identifier for the cluster 
  unsigned long int age; //age of the cluster 
  float r, g, b, a; //color of the cluster

  visualization_msgs::Marker getBoundingBoxCenterVisualisationMessage();
  visualization_msgs::Marker getClosestCornerPointVisualisationMessage();
  visualization_msgs::Marker getClusterVisualisationMessage();
  visualization_msgs::Marker getLineVisualisationMessage();
  visualization_msgs::Marker getArrowVisualisationMessage();
  visualization_msgs::Marker getThetaL2VisualisationMessage();
  visualization_msgs::Marker getThetaL1VisualisationMessage();
  visualization_msgs::Marker getThetaBoxVisualisationMessage();
  visualization_msgs::Marker getBoundingBoxVisualisationMessage();
  visualization_msgs::Marker getBoxModelVisualisationMessage();
  visualization_msgs::Marker getLShapeVisualisationMessage();
  visualization_msgs::Marker getPoseCovariance();

  pair<int, int> getRectangleFittingExecutionTime(){return dur_size_rectangle_fitting;};
  pair<int, int> dur_size_rectangle_fitting;

  void update(const pointList&, const double dt, const tf::Transform& ego_pose);
  void populateTrackingMsgs();
  void detectCornerPointSwitch();
  void detectCornerPointSwitch(double& from, double& to);
  bool red_flag, green_flag, blue_flag;

  std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.

  double meanX() { return mean_values.first; };
  double meanY() { return mean_values.second;};

  LShapeTracker l_shape; 
  KalmanFilter kf_mean;
  double old_thetaL1, old_thetaL2;
  double L1, L2, thetaL1, thetaL2;
  double cx, cy, L1_box, L2_box, th; 

private:

  pointList new_cluster;
  vector<Point> corner_list;

  vector<Point> l1l2; //save coordinates of the three points that define the lines

  // mean value of the cluster
  std::pair<double, double> mean_values;
  std::pair<double, double> previous_mean_values;

  Point closest_corner_point;
  
  visualization_msgs::Marker boxcenter_marker_;
  void calcMean(const pointList& ); //Find the mean value of the cluster
  void rectangleFitting(const pointList& ); //Search-Based Rectangle Fitting 
  double areaCriterion(const VectorXd&, const VectorXd& );
  double closenessCriterion(const VectorXd& ,const VectorXd&, const float& );
  Point lineIntersection(double& , double& , double& , double& , double& , double& );
  double perpendicularDistance(const Point&, const Point&, const Point&);
  void ramerDouglasPeucker(const vector<Point>&, double, vector<Point>&);
};
