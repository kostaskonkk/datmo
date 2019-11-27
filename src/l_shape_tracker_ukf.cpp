/**
 * Implementation of L-Shape Tracker class with a UKF Filter.
*
* @author: Kostas Konstantinidis
* @date: 26.11.2019
*/

#include "l_shape_tracker_ukf.hpp"

LShapeTrackerUKF::LShapeTrackerUKF(){}//Creates a blank estimator

LShapeTrackerUKF::LShapeTrackerUKF(const RobotLocalization::Ukf& ukf, const double& L1, const double& L2, const double& theta, const double& dt){

  this->ukf = ukf;

  // Initialization of Shape Kalman Filter
  int n = 4; // Number of states
  int m = 3; // Number of measurements
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

void LShapeTrackerUKF::update(const RobotLocalization::Measurement& measurement, const double& L1, const double& L2, const double& theta, const double& dt) {


  ukf.correct_ctrm(measurement);
  ukf.predict_ctrm(dt);

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
void LShapeTrackerUKF::ClockwisePointSwitch(){
  // Equation 17

  const double pi = 3.141592653589793238463; 
  
  Vector4d new_shape_states;
  VectorXd new_ukf_states = ukf.getState();

  double L1 = shape_kf.state()(0);
  double L2 = shape_kf.state()(1);

  //x = x + L1 * cos(theta);
  new_ukf_states(X)  = ukf.getState()(X) + L1 * cos(ukf.getState()(Yaw));
  //y = y + L1 * sin(theta);
  new_ukf_states(Y)  = ukf.getState()(Y) + L1 * sin(ukf.getState()(Yaw));
  //vx = vx - L1 * omega * sin(theta);
  new_ukf_states(Vx) = ukf.getState()(Vx)- L1 * ukf.getState()(Vyaw) * sin(ukf.getState()(Yaw));
  //vy = vy + L1 * omega * cos(theta);
  new_ukf_states(Vy) = ukf.getState()(Vy)+ L1 * ukf.getState()(Vyaw) * cos(ukf.getState()(Yaw));
  new_ukf_states(Yaw) = ukf.getState()(Yaw) - pi / 2;

  //L1 = L2
  new_shape_states(0) = L2;
  //L2 = L1
  new_shape_states(1) = L1;

  ukf.setState(new_ukf_states);
  shape_kf.changeStates(new_shape_states);

}
void LShapeTrackerUKF::CounterClockwisePointSwitch(){
  // Equation 17

  const double pi = 3.141592653589793238463; 
  
  Vector4d new_shape_states;
  VectorXd new_ukf_states = ukf.getState();

  double L1 = shape_kf.state()(0);
  double L2 = shape_kf.state()(1);

  //x = x + L2 * sin(theta);
  new_ukf_states(X)  = ukf.getState()(X) + L2 * sin(ukf.getState()(Yaw));
  //y = y - L2 * cos(theta);
  new_ukf_states(Y)  = ukf.getState()(Y) - L2 * cos(ukf.getState()(Yaw));
  //vx = vx + L2 * omega * cos(theta);
  new_ukf_states(Vx) = ukf.getState()(Vx)+ L2 * ukf.getState()(Vyaw) * cos(ukf.getState()(Yaw));
  //vy = vy + L2 * omega * sin(theta);
  new_ukf_states(Vy) = ukf.getState()(Vy)+ L2 * ukf.getState()(Vyaw) * sin(ukf.getState()(Yaw));
  new_ukf_states(Yaw) = ukf.getState()(Yaw) + pi / 2;

  //L1 = L2
  new_shape_states(0) = L2;
  //L2 = L1
  new_shape_states(1) = L1;

  ukf.setState(new_ukf_states);
  shape_kf.changeStates(new_shape_states);

}

void LShapeTrackerUKF::lshapeToBoxModelConversion(double& x, double& y,double& vx, double& vy, double& L1, double& L2, double& theta, double& omega){
  L1 = shape_kf.state()(0);
  L2 = shape_kf.state()(1);
  theta = ukf.getState()(Yaw);
  omega = ukf.getState()(Vyaw);
  //Equations 30 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  double ex = (L1 * cos(theta) + L2 * sin(theta)) /2;
  double ey = (L1 * sin(theta) - L2 * cos(theta)) /2;
  x = ukf.getState()(X) + ex;
  y = ukf.getState()(Y) + ey;

  //Equations 31 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  //TODO test the complete equation also
  vx = ukf.getState()(Vx);
  vy = ukf.getState()(Vy);

}

double LShapeTrackerUKF::findTurn(double& new_angle, double& old_angle){
  //https://math.stackexchange.com/questions/1366869/calculating-rotation-direction-between-two-angles
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

void LShapeTrackerUKF::detectCornerPointSwitch(double& from, double& to){
  //Corner Point Switch Detection
  
  double turn = this->findTurn(from, to);
    if(turn <-0.6){
     this->CounterClockwisePointSwitch();
    }
    else if(turn > 0.6){
     this->ClockwisePointSwitch();
    }

}
