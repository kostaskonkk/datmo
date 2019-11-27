#pragma once

#include "kalman-cpp/kalman.hpp"
#include <Eigen/Dense>
//#include <math.h>       [> atan2 <]

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;
typedef Eigen::Matrix< double, 6, 1 > Vector6d;

class LShapeTracker {
public:

  LShapeTracker(const Point& ,const double& ,const double& ,const double& ,const double& );
  LShapeTracker();//Create a blank estimator

  void update(const Point& ,const double& ,const double& ,const double& ,const double& );

  void lshapeToBoxModelConversion(double& x, double& y, double& vx, double& vy, double& L1, double& L2, double& th, double& omega);
  void ClockwisePointSwitch();
  void CounterClockwisePointSwitch();
  void changeStates(const Vector6d& new_dynamic_states, const Vector4d& new_shape_states);
  double findTurn(double& new_angle, double& old_angle);
  void detectCornerPointSwitch(double& from, double& to);

  KalmanFilter dynamic_kf;
  KalmanFilter shape_kf;


private:

};
