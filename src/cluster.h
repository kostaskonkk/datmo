#pragma once

#include <vector>
#include <random>
#include <visualization_msgs/Marker.h>
#include "kalman-cpp/kalman.hpp"
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;
typedef std::vector<Point> pointList;


class Cluster {

public:

  unsigned long int id; //identifier for the filter

  float g, r, b; //randomly assigned color to the cluster

  Cluster(unsigned long int id, const pointList& );

  visualization_msgs::Marker getPointVisualisationMessage();

  visualization_msgs::Marker getSavedClustersVisualisationMessage();

  visualization_msgs::Marker getClusterVisualisationMessage();

  visualization_msgs::Marker getLineVisualisationMessage();

  visualization_msgs::Marker getArrowVisualisationMessage();

  visualization_msgs::Marker getSavedClusterVisualisationMessage();


  geometry_msgs::Pose getPose();


  void update(const pointList& );

  std::pair<double, double> mean() { return mean_values; }; //Return the mean of the cluster.

  double mean_x() { return mean_values.first; };

  double mean_y() { return mean_values.second;};

  int size() { return clusters.size(); };

  double PerpendicularDistance(const Point&, const Point&, const Point&);

  void RamerDouglasPeucker(const vector<Point>&, double, vector<Point>&);

  // float lineSegmentExtractor(const vector<Point> &, vector<double> &, bool );

private:

  vector<pointList> clusters;

  // mean value of the cluster
  std::pair<double, double> mean_values;
  double theta;
  // Discrete time step
  double dt;

  // Is the filter initialized?
  bool initialized;

  void calc_mean(const pointList& ); //Find the mean value of the cluster

  void calc_theta();

};
