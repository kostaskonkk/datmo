#pragma once

#include "kalman-cpp/kalman.hpp"
#include <Eigen/Dense>
#include <ros/console.h>
#include "ukf/ukf.h"

using namespace Eigen;

typedef std::pair<double, double> Point;

enum StateMembers
{
  X,
  Y,
  Vx,
  Vy,
  Vyaw,
  //Yaw,
};

class LshapeTracker {
public:


  LshapeTracker(const double& x_corner, const double& y_corner, const double& L1, const double& L2, const double& theta, const double& dt);
  LshapeTracker();//Create a blank estimator

  void update(const double& old_thetaL1, const double& thetaL1, const double& x_corner, const double& y_corner, const double& L1, const double& L2, const double& theta, const double& dt);
  //void updateDynamic(const RobotLocalization::Measurement& measurement, const double& dt);

  void lshapeToBoxModelConversion(double& x, double& y, double& vx, double& vy, double& L1, double& L2, double& theta, double& psi, double& omega);

  void ClockwisePointSwitch();
  void CounterClockwisePointSwitch();
  //void changeStates(const Eigen::Vector4d& new_dynamic_states, const Eigen::Vector3d& new_shape_states);
  double findTurn(const double& new_angle, const double& old_angle);
  void detectCornerPointSwitch(const double& from, const double& to, const double dt);

  RobotLocalization::Ukf ukf;
  KalmanFilter shape_kf;

private:



};
