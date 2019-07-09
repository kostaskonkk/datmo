#pragma once

#include "kalman-cpp/kalman.hpp"
#include "l_shape_tracker.h"
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
//#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "datmo/Track.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;


class Cluster {
public:

  Cluster(unsigned long int id, const pointList&, const double&, const tf::TransformListener& );

  nav_msgs::Path trajectory_;

  string p_target_frame_name_ = "map";
  string p_source_frame_name_ = "laser";

  datmo::Track track_msg;
  datmo::Track filtered_track_msg;

  // Poses used for transformation to target_frame.
  geometry_msgs::PoseStamped pose_source_;

  unsigned long int id; //identifier for the cluster 

  float r, g, b; //current color of the cluster

  geometry_msgs::Quaternion geo;//added for debug

  visualization_msgs::Marker getCenterVisualisationMessage();
  visualization_msgs::Marker getClosestCornerPointVisualisationMessage();
  visualization_msgs::Marker getClusterVisualisationMessage();
  visualization_msgs::Marker getLineVisualisationMessage();
  visualization_msgs::Marker getArrowVisualisationMessage();
  visualization_msgs::Marker getBoundingBoxVisualisationMessage();
  visualization_msgs::Marker getBoxVisualisationMessage();
  visualization_msgs::Marker getL1L2VisualisationMessage();

  nav_msgs::Path getTrajectory();

  void update(const pointList& , const double, const tf::TransformListener& );

  std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.

  double meanX() { return mean_values.first; };
  double meanY() { return mean_values.second;};


  KalmanFilter kf;
  KalmanFilter map_kf;

  double avx, avy; //for test
private:
  bool moving; 

  //vector<pointList> clusters;
  pointList new_cluster;
  vector<Point> corner_list;

  vector<Point> l1l2; //save coordinates of the three points that define the lines

  // mean value of the cluster
  std::pair<double, double> mean_values;
  std::pair<double, double> previous_mean_values;
  std::pair<double, double> abs_mean_values;
  std::pair<double, double> abs_previous_mean_values;

  Point closest_corner_point;
  
  double dt; // Discrete time step

  double vx, vy;

  float L1, L2, theta;

  void calcMean(const pointList& ); //Find the mean value of the cluster
  void rectangleFitting(const pointList& ); //Search-Based Rectangle Fitting 
  double areaCriterion(const VectorXd&, const VectorXd& );
  double closenessCriterion(const VectorXd& ,const VectorXd&, const float& );
  Point lineIntersection(double& , double& , double& , double& , double& , double& );
  double perpendicularDistance(const Point&, const Point&, const Point&);
  void ramerDouglasPeucker(const vector<Point>&, double, vector<Point>&);

};
