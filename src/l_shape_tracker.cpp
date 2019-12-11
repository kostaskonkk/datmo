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
  int n = 6; // Number of states
  int m = 2; // Number of measurements
  MatrixXd A(n, n); // System dynamics matrix
  MatrixXd C(m, n); // Output matrix
  MatrixXd Q(n, n); // Process noise covariance
  MatrixXd R(m, m); // Measurement noise covariance
  MatrixXd P(n, n); // Estimate error covariance
      
  //A << 1, 0,dt, 0, 
       //0, 1, 0,dt, 
       //0, 0, 1, 0, 
       //0, 0, 0, 1;
       //
  double ddt = dt*dt/2;
  A << 1, 0,dt, 0, ddt,   0, 
       0, 1, 0,dt,   0, ddt, 
       0, 0, 1, 0,  dt,   0, 
       0, 0, 0, 1,   0,  dt,
       0, 0, 0, 0,   1,   0, 
       0, 0, 0, 0,   0,   1;

  C << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0;

  Q << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0,10, 0, 0, 0,
       0, 0, 0,10, 0, 0,
       0, 0, 0, 0,20, 0,
       0, 0, 0, 0, 0,20;
  //Q.setIdentity();
  R.setIdentity();
  R *= 0.1;
  P.setIdentity();

  KalmanFilter dynamic_kalman_filter(dt, A, C, Q, R, P); 
  this->dynamic_kf = dynamic_kalman_filter;

  VectorXd x0_dynamic(n);
  x0_dynamic << corner_point.first, corner_point.second, 0, 0, 0, 0;
  dynamic_kf.init(0,x0_dynamic);

  // Initialization of Shape Kalman Filter
  n = 4; // Number of states
  m = 3; // Number of measurements
  MatrixXd As(n, n); // System dynamics matrix
  MatrixXd Cs(m, n); // Output matrix
  MatrixXd Qs(n, n); // Process noise covariance
  MatrixXd Rs(m, m); // Measurement noise covariance
  MatrixXd Ps(n, n); // Estimate error covariance
      
  As<< 1, 0, 0, 0, 
       0, 1, 0, 0, 
       0, 0, 1,dt, 
       0, 0, 0, 1;

  Cs<< 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1, 0;

  Qs.setIdentity();
  Rs.setIdentity();
  Ps.setIdentity();

  KalmanFilter shape_kalman_filter(dt, As, Cs, Qs, Rs, Ps); 
  this->shape_kf = shape_kalman_filter;

  VectorXd x0_shape(n);
  x0_shape << L1, L2, theta, 0;
  shape_kf.init(0,x0_shape);
  
}

void LShapeTracker::update(const Point& corner_point, const double& L1, const double& L2, const double& theta, const double& dt) {

  // Update Dynamic Kalman Filter
  Vector2d y;
  y << corner_point.first, corner_point.second;
  dynamic_kf.update(y, dt);

  // Update Shape Kalman Filter
  Vector3d shape_measurements;
  double L1max, L2max;
  if(L1 > shape_kf.state()(0)){
    L1max = L1;}
  else{
    L1max = shape_kf.state()(0);}
  if(L2 > shape_kf.state()(1)){
    L2max = L2;}
  else{
    L2max = shape_kf.state()(1);}
  shape_measurements << L1max, L2max, theta;
  shape_kf.update(shape_measurements, dt);

}
void LShapeTracker::ClockwisePointSwitch(){
  // Equation 17

  const double pi = 3.141592653589793238463; 
  
  Vector6d new_dynamic_states;
  Vector4d new_shape_states;
  new_dynamic_states = dynamic_kf.state();
  new_shape_states = shape_kf.state();
  //x = x + L1 * cos(theta);
  new_dynamic_states(0) += shape_kf.state()(0) * cos(shape_kf.state()(2));
  //y = y + L1 * sin(theta);
  new_dynamic_states(1) += shape_kf.state()(0) * sin(shape_kf.state()(2));
  //vx = vx - L1 * omega * sin(theta);
  new_dynamic_states(2) -= shape_kf.state()(0) * shape_kf.state()(3) *  sin(shape_kf.state()(2));
  //vy = vy + L1 * omega * cos(theta);
  new_dynamic_states(3) += shape_kf.state()(0) * shape_kf.state()(3) *  cos(shape_kf.state()(2));
  //ax = ax - L1 * omega^2 * cos(theta);
  new_dynamic_states(4) -= shape_kf.state()(0) * pow(shape_kf.state()(3),2) *  cos(shape_kf.state()(2));
  //ay = ay - L1 * omega^2 * sin(theta);
  new_dynamic_states(5) -= shape_kf.state()(0) * pow(shape_kf.state()(3),2) *  sin(shape_kf.state()(2));
  //L1 = L2
  new_shape_states(0) = shape_kf.state()(1);
  //L2 = L1
  new_shape_states(1) = shape_kf.state()(0);

  new_shape_states(2) = shape_kf.state()(2) - pi / 2;

  dynamic_kf.changeStates(new_dynamic_states);
  shape_kf.changeStates(new_shape_states);
}
void LShapeTracker::CounterClockwisePointSwitch(){
  // Equation 17

  const double pi = 3.141592653589793238463; 
  
  Vector6d new_dynamic_states;
  Vector4d new_shape_states;
  new_dynamic_states = dynamic_kf.state();
  new_shape_states = shape_kf.state();
  //x = x + L2 * sin(theta);
  new_dynamic_states(0) += shape_kf.state()(1) * sin(shape_kf.state()(2));
  //y = y - L2 * cos(theta);
  new_dynamic_states(1) -= shape_kf.state()(1) * cos(shape_kf.state()(2));
  //vx = vx + L2 * omega * cos(theta);
  new_dynamic_states(2) += shape_kf.state()(1) * shape_kf.state()(3) *  cos(shape_kf.state()(2));
  //vy = vy + L2 * omega * sin(theta);
  new_dynamic_states(3) +=  shape_kf.state()(1) * shape_kf.state()(3) *  sin(shape_kf.state()(2));
  //ax = ax - L2 * omega^2 * cos(theta);
  new_dynamic_states(4) = dynamic_kf.state()(4) - shape_kf.state()(1) * pow(shape_kf.state()(3),2) *  sin(shape_kf.state()(2));
  //ay = ay - L2 * omega^2 * sin(theta);
  new_dynamic_states(5) = dynamic_kf.state()(5) + shape_kf.state()(1) * pow(shape_kf.state()(3),2) *  cos(shape_kf.state()(2));

  //L1 = L2
  new_shape_states(0) = shape_kf.state()(1);
  //L2 = L1
  new_shape_states(1) = shape_kf.state()(0);
  new_shape_states(2) = shape_kf.state()(2) + pi / 2;

  dynamic_kf.changeStates(new_dynamic_states);
  shape_kf.changeStates(new_shape_states);
}

void LShapeTracker::lshapeToBoxModelConversion(double& x, double& y,double& vx, double& vy, double& L1, double& L2, double& theta, double& omega){
  L1 = shape_kf.state()(0);
  L2 = shape_kf.state()(1);
  theta = shape_kf.state()(2);
  omega = shape_kf.state()(3);
  //Equations 30 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  double ex = (L1 * cos(theta) + L2 * sin(theta)) /2;
  double ey = (L1 * sin(theta) - L2 * cos(theta)) /2;
  x = dynamic_kf.state()(0) + ex;
  y = dynamic_kf.state()(1) + ey;

  //Equations 31 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  //TODO test the complete equation also
  vx = dynamic_kf.state()(2);
  vy = dynamic_kf.state()(3);

}

double LShapeTracker::findTurn(double& new_angle, double& old_angle){
  //https://math.stackexchange.com/questions/1366869/calculating-rotation-direction-between-two-angles
  //const double pi = 3.141592653589793238463; 
  double theta_pro = new_angle - old_angle;
  double turn = 0;
  if(-M_PI<=theta_pro && theta_pro <= M_PI){
    turn = theta_pro;}
  else if(theta_pro > M_PI){
    turn = theta_pro - 2*M_PI;}
  else if(theta_pro < -M_PI){
    turn = theta_pro + 2*M_PI;}
  return turn;
}

void LShapeTracker::detectCornerPointSwitch(double& from, double& to){
  //Corner Point Switch Detection
  
  double turn = this->findTurn(from, to);
    if(turn <-0.8){
     this->CounterClockwisePointSwitch();
     //ROS_INFO_STREAM("from: "<<from<<"to: "<<to);
    }
    else if(turn > 0.6){
     this->ClockwisePointSwitch();
    }

}
