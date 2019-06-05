#pragma once

//#include "trajectory.h"
#include "kalman-cpp/kalman.hpp"
#include <vector>
#include <random>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "tf/transform_listener.h"

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;


class Cluster {
public:

  //void addPoseToTrajectory(const geometry_msgs::PoseStamped& );

  //void waitForTf();

//  nav_msgs::Path getTrajectory();
  //parameters

  void updateTrajectory(const tf::TransformListener& );
  nav_msgs::Path trajectory_;

  std::string p_target_frame_name_ = "map";
  std::string p_source_frame_name_ = "laser";

  double p_trajectory_update_rate_;
  double p_trajectory_publish_rate_;

  // Poses used for transformation to target_frame.
  geometry_msgs::PoseStamped pose_source_;

  unsigned long int id; //identifier for the filter

  float g, r, b; //randomly assigned color to the cluster

  Cluster(unsigned long int id, const pointList&, const double& );

  visualization_msgs::Marker getPointVisualisationMessage();
  visualization_msgs::Marker getClusterVisualisationMessage();
  visualization_msgs::Marker getLineVisualisationMessage();
  visualization_msgs::Marker getArrowVisualisationMessage();
  visualization_msgs::Marker getBoundingBoxVisualisationMessage();

  nav_msgs::Path getTrajectory();

  geometry_msgs::Pose getPose();

  geometry_msgs::Pose getVel();

  nav_msgs::Odometry getOdom();

  nav_msgs::Odometry getFilteredOdom();


  void update(const pointList& , const double);

  std::pair<double, double> mean() { return mean_values; }; //Return mean of cluster.

  double meanX() { return mean_values.first; };

  double meanY() { return mean_values.second;};

  int size() { return clusters.size(); };

  // float lineSegmentExtractor(const vector<Point> &, vector<double> &, bool );

private:
  bool moving; 

  KalmanFilter kf;

  vector<pointList> clusters;

  // mean value of the cluster
  std::pair<double, double> mean_values;
  std::pair<double, double> previous_mean_values;

  double theta;
  // Discrete time step
  double dt;
  // Is the filter initialized?
  bool initialized;

  double vx, vy;
  float Lx, Ly;

  void calcMean(const pointList& ); //Find the mean value of the cluster

  void calcRelativeVelocity();

  void calcTheta();

  double perpendicularDistance(const Point&, const Point&, const Point&);

  void ramerDouglasPeucker(const vector<Point>&, double, vector<Point>&);

};
