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
  Yaw,
  Vx,
  Vy,
  Vyaw,
};

class LShapeTrackerUKF {
public:


  LShapeTrackerUKF(const RobotLocalization::Ukf& ukf, const double& L1, const double& L2, const double& theta, const double& dt);
  LShapeTrackerUKF();//Create a blank estimator

  void update(const RobotLocalization::Measurement& measurement, const double& L1, const double& L2, const double& theta, const double& dt);

  void lshapeToBoxModelConversion(double& x, double& y, double& vx, double& vy, double& L1, double& L2, double& theta, double& psi, double& omega);

  void ClockwisePointSwitch();
  void CounterClockwisePointSwitch();
  //void changeStates(const Eigen::Vector4d& new_dynamic_states, const Eigen::Vector3d& new_shape_states);
  double findTurn(double& new_angle, double& old_angle);
  void detectCornerPointSwitch(double& from, double& to, const double dt);

  RobotLocalization::Ukf ukf;
  KalmanFilter shape_kf;

private:



};
