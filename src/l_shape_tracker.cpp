/**
 * Implementation of L-Shape Tracker class.
*
* @author: Kostas Konstantinidis
* @date: 05.07.2019
*/

#include "l_shape_tracker.hpp"

LShapeTracker::LShapeTracker(){}//Creates a blank estimator

LShapeTracker::LShapeTracker(const Point& corner_point, const double& L1, const double& L2, const double& theta, const double& dt){


  // Initialization of Dynamic Kalman Filter
  int n = 4; // Number of states
  int m = 2; // Number of measurements
  MatrixXd A(n, n); // System dynamics matrix
  MatrixXd C(m, n); // Output matrix
  MatrixXd Q(n, n); // Process noise covariance
  MatrixXd R(m, m); // Measurement noise covariance
  MatrixXd P(n, n); // Estimate error covariance
      
  A << 1, 0,dt, 0, 
       0, 1, 0,dt, 
       0, 0, 1, 0, 
       0, 0, 0, 1;

  C << 1, 0, 0, 0,
       0, 1, 0, 0;

  Q.setIdentity();
  R.setIdentity();
  P.setIdentity();

  KalmanFilter dynamic_kalman_filter(dt, A, C, Q, R, P); 
  this->dynamic = dynamic_kalman_filter;

  VectorXd x0_dynamic(n);
  x0_dynamic << corner_point.first, corner_point.second, 0, 0;
  dynamic.init(0,x0_dynamic);

  // Initialization of Shape Kalman Filter
  n = 4; // Number of states
  n = 3; // Number of states
  m = 3; // Number of measurements
  MatrixXd As(n, n); // System dynamics matrix
  MatrixXd Cs(m, n); // Output matrix
  MatrixXd Qs(n, n); // Process noise covariance
  MatrixXd Rs(m, m); // Measurement noise covariance
  MatrixXd Ps(n, n); // Estimate error covariance
      
  //As<< 1, 0, 0, 0, 
       //0, 1, 0, 0, 
       //0, 0, 1,dt, 
       //0, 0, 0, 1;

  As<< 1, 0, 0, 
       0, 1, 0, 
       0, 0, 1; 

  //Cs<< 1, 0, 0, 0,
       //0, 1, 0, 0,
       //0, 0, 1, 0;
  Cs<< 1, 0, 0,
       0, 1, 0,
       0, 0, 1;

  Qs.setIdentity();
  Rs.setIdentity();
  Ps.setIdentity();

  KalmanFilter shape_kalman_filter(dt, As, Cs, Qs, Rs, Ps); 
  this->shape = shape_kalman_filter;

  VectorXd x0_shape(n);
  //x0_shape << L1, L2, theta, 0;
  x0_shape << L1, L2, theta;
  shape.init(0,x0_shape);
  
  
}

void LShapeTracker::update(const Point& corner_point, const double& L1, const double& L2, const double& theta, const double& dt) {

  // Update Dynamic Kalman Filter
  VectorXd y(2);
  y << corner_point.first, corner_point.second;
  dynamic.update(y, dt);

  // Update Shape Kalman Filter
  VectorXd y_shape(3);
  double L1max, L2max;
  if(L1 > shape.state()(0)){
    L1max = L1;}
  else{
    L1max = shape.state()(0);}
  if(L2 > shape.state()(1)){
    L2max = L2;}
  else{
    L2max = shape.state()(1);}
  y_shape << L1max, L2max, theta;
  shape.update(y_shape, dt);

}
void LShapeTracker::ClockwisePointSwitch(){
  // Equation 17

  const double pi = 3.141592653589793238463; 
  
  Vector4d new_dynamic_states;
  Vector3d new_shape_states;
  new_dynamic_states = dynamic.state();
  new_shape_states = shape.state();
  //x = x + L1 * cos(theta);
  new_dynamic_states(0) = dynamic.state()(0) + shape.state()(0) * cos(shape.state()(2));
  //y = y + L1 * sin(theta);
  new_dynamic_states(1) = dynamic.state()(1) + shape.state()(0) * sin(shape.state()(2));
  //L1 = L2
  new_shape_states(0) = shape.state()(1);
  //L2 = L1
  new_shape_states(1) = shape.state()(0);
  new_shape_states(2) = shape.state()(2) - pi / 2;

  dynamic.changeStates(new_dynamic_states);
  shape.changeStates(new_shape_states);
}
void LShapeTracker::CounterClockwisePointSwitch(){
  // Equation 17

  const double pi = 3.141592653589793238463; 
  
  Vector4d new_dynamic_states;
  Vector3d new_shape_states;
  new_dynamic_states = dynamic.state();
  new_shape_states = shape.state();
  //x = x + L1 * cos(theta);
  new_dynamic_states(0) = dynamic.state()(0) + shape.state()(1) * sin(shape.state()(2));
  //y = y + L1 * sin(theta);
  new_dynamic_states(1) = dynamic.state()(1) - shape.state()(1) * cos(shape.state()(2));
  //L1 = L2
  new_shape_states(0) = shape.state()(1);
  //L2 = L1
  new_shape_states(1) = shape.state()(0);
  new_shape_states(2) = shape.state()(2) + pi / 2;

  dynamic.changeStates(new_dynamic_states);
  shape.changeStates(new_shape_states);
}

void LShapeTracker::changeStates(const Eigen::Vector4d& new_dynamic_states,const Eigen::Vector3d& new_shape_states ){
  dynamic.changeStates(new_dynamic_states);
  shape.changeStates(new_shape_states);
}

void LShapeTracker::lshapeToBoxModelConversion(double& x, double& y, double& L1, double& L2, double& th){
  //Equations 30 and 31 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  th = shape.state()(2);
  L1 = shape.state()(0);
  L2 = shape.state()(1);
  double ex = (L1 * cos(th) + L2 * sin(th)) /2;
  double ey = (L1 * sin(th) - L2 * cos(th)) /2;
  x = dynamic.state()(0) + ex;
  y = dynamic.state()(1) + ey;

}
