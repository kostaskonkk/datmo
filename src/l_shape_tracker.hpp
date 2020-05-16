#pragma once

#include "kalman-cpp/kalman.hpp"
#include <Eigen/Dense>
#include <ros/console.h>

using namespace Eigen;

typedef std::pair<double, double> Point;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

const double pi = 3.141592653589793238463; 

class LshapeTracker {
public:

  LshapeTracker(const double& x_corner, const double& y_corner, const double& L1, const double& L2, const double& theta, const double& dt);
  LshapeTracker();
  void update(const double& thetaL1, const double& x_corner, const double& y_corner, const double& L1, const double& L2, const double& dt, const int cluster_size);
  void BoxModel(double& x, double& y,double& vx, double& vy,double& theta, double& psi, double& omega, double& L1, double& L2, double& length, double& width);

private:
  int current_size;
  double test1, test2, test3;
  double x_old, y_old, L1_old, L2_old, old_thetaL1;

  KalmanFilter shape_kf;
  KalmanFilter dynamic_kf;

  void ClockwisePointSwitch();
  void CounterClockwisePointSwitch();
  double findTurn(const double& new_angle, const double& old_angle);
  void detectCornerPointSwitch(const double& from, const double& to, const double dt);
  void detectCornerPointSwitchMahalanobis(const double& from, const double& to, const double dt);
  void detectCornerPointSwitchMahalanobis(const double& from, const double& to, const double L1, const double L2, const double x_corner, const double y_corner);
  /*! \brief Finds orientations of tracked object
   *
   * Given the orientation of L1 the other three angles of the rectangle are calculated.
   * Then they are compared with the speed of the object, to estimate it's direction.
   *
   * \angle angle of one edge of the box
   * \vx velocity in the x axis
   * \vy velocity in the y axis
   * \orientation orientation of tracked object, based on it's speed
   */
  void findOrientation(double& psi, double& length, double& width);
};
