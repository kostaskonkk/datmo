#pragma once

#include "kalman-cpp/kalman.hpp"
#include <Eigen/Dense>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "datmo/Track.h"

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;


class Cluster {
public:

  Cluster(unsigned long int id, const pointList&, const double&, const tf::TransformListener& );

  nav_msgs::Path trajectory_;

  std::string p_target_frame_name_ = "map";
  std::string p_source_frame_name_ = "laser";

  double p_trajectory_update_rate_;
  double p_trajectory_publish_rate_;

  datmo::Track track_msg;
  datmo::Track filtered_track_msg;

  // Poses used for transformation to target_frame.
  geometry_msgs::PoseStamped pose_source_;

  unsigned long int id; //identifier for the cluster 

  //float rr, rg, rb; //randomly assigned color to the cluster
  float r, g, b; //current color of the cluster


  visualization_msgs::Marker getCenterVisualisationMessage();
  visualization_msgs::Marker getClusterVisualisationMessage();
  visualization_msgs::Marker getLineVisualisationMessage();
  visualization_msgs::Marker getArrowVisualisationMessage();
  visualization_msgs::Marker getBoundingBoxVisualisationMessage();

  nav_msgs::Path getTrajectory();

  void update(const pointList& , const double, const tf::TransformListener& );

  std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.

  double meanX() { return mean_values.first; };

  double meanY() { return mean_values.second;};

  int size() { return clusters.size(); };

  // float lineSegmentExtractor(const vector<Point> &, vector<double> &, bool );

  KalmanFilter kf;
  KalmanFilter map_kf;

  double avx, avy; //for test
private:
  bool moving; 

  vector<pointList> clusters;

  // mean value of the cluster
  std::pair<double, double> mean_values;
  std::pair<double, double> previous_mean_values;
  std::pair<double, double> abs_mean_values;
  std::pair<double, double> abs_previous_mean_values;

  double theta;
  
  double dt; // Discrete time step
  
  double vx, vy;
  float Lx, Ly;

  void calcMean(const pointList& ); //Find the mean value of the cluster

  void calcTheta();

  double perpendicularDistance(const Point&, const Point&, const Point&);

  void ramerDouglasPeucker(const vector<Point>&, double, vector<Point>&);

};
