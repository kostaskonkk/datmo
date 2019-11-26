#pragma once

#include <Eigen/Dense>
#include "ukf/ukf.h"

using namespace Eigen;

typedef std::pair<double, double> Point;

class LShapeTrackerUKF {
public:

  RobotLocalization::Ukf ukf;

  LShapeTrackerUKF(const RobotLocalization::Ukf& ukf);
  LShapeTrackerUKF();//Create a blank estimator

  void update(const RobotLocalization::Measurement& measurement, const double& dt);

  void lshapeToBoxModelConversion(double& x, double& y, double& vx, double& vy, double& L1, double& L2, double& th, double& omega);

  void ClockwisePointSwitch();
  void CounterClockwisePointSwitch();
  void changeStates(const Eigen::Vector4d& new_dynamic_states, const Eigen::Vector3d& new_shape_states);

private:


};
