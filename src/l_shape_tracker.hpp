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

  LShapeTracker(const Point& corner_point, const double& L1, const double& L2, const double& theta, const double& dt);
  LShapeTracker();//Create a blank estimator

  void update(const Point& corner_point, const double& L1, const double& L2, const double& theta, const double& dt);
  void updateShape(const double& L1, const double& L2, const double& theta, const double& dt);
  void updateDynamic(const Point& corner_point, const double& dt); 

  void lshapeToBoxModelConversion(double& x, double& y, double& vx, double& vy, double& L1, double& L2, double& th, double& omega);
  void ClockwisePointSwitch();
  void CounterClockwisePointSwitch();
  double findTurn(double& new_angle, double& old_angle);
  void detectCornerPointSwitch(double& from, double& to);

  KalmanFilter dynamic_kf;
  KalmanFilter shape_kf;


private:

};
