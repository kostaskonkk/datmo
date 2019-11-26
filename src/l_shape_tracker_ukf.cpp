/**
 * Implementation of L-Shape Tracker class with a UKF Filter.
*
* @author: Kostas Konstantinidis
* @date: 26.11.2019
*/

#include "l_shape_tracker_ukf.hpp"

LShapeTrackerUKF::LShapeTrackerUKF(){}//Creates a blank estimator

LShapeTrackerUKF::LShapeTrackerUKF(const RobotLocalization::Ukf& ukf){

  this->ukf = ukf;

}

void LShapeTrackerUKF::update(const RobotLocalization::Measurement& measurement, const double& dt) {


  ukf.correct_ctrm(measurement);
  ukf.predict_ctrm(dt);

}
void LShapeTrackerUKF::ClockwisePointSwitch(){
  // Equation 17

  const double pi = 3.141592653589793238463; 
  
  Vector4d new_dynamic_states;
  Vector4d new_shape_states;
  //new_dynamic_states = dynamic_kf.state();
  //new_shape_states = shape_kf.state();
  //x = x + L1 * cos(theta);
  //new_dynamic_states(0) = dynamic_kf.state()(0) + shape_kf.state()(0) * cos(shape_kf.state()(2));
  //y = y + L1 * sin(theta);
  //new_dynamic_states(1) = dynamic_kf.state()(1) + shape_kf.state()(0) * sin(shape_kf.state()(2));
  //L1 = L2
  //new_shape_states(0) = shape_kf.state()(1);
  //L2 = L1
  //new_shape_states(1) = shape_kf.state()(0);

  //new_shape_states(2) = shape_kf.state()(2) - pi / 2;

  //dynamic_kf.changeStates(new_dynamic_states);
  //shape_kf.changeStates(new_shape_states);
}
void LShapeTrackerUKF::CounterClockwisePointSwitch(){
  //// Equation 17

  //const double pi = 3.141592653589793238463; 
  
  //Vector4d new_dynamic_states;
  //Vector4d new_shape_states;
  //new_dynamic_states = dynamic_kf.state();
  //new_shape_states = shape_kf.state();
  ////x = x + L1 * cos(theta);
  //new_dynamic_states(0) = dynamic_kf.state()(0) + shape_kf.state()(1) * sin(shape_kf.state()(2));
  ////y = y + L1 * sin(theta);
  //new_dynamic_states(1) = dynamic_kf.state()(1) - shape_kf.state()(1) * cos(shape_kf.state()(2));
  ////L1 = L2
  //new_shape_states(0) = shape_kf.state()(1);
  ////L2 = L1
  //new_shape_states(1) = shape_kf.state()(0);
  //new_shape_states(2) = shape_kf.state()(2) + pi / 2;

  //dynamic_kf.changeStates(new_dynamic_states);
  //shape_kf.changeStates(new_shape_states);
//}

//void LShapeTrackerUKF::changeStates(const Eigen::Vector4d& new_dynamic_states,const Eigen::Vector3d& new_shape_states ){
  //dynamic_kf.changeStates(new_dynamic_states);
  //shape_kf.changeStates(new_shape_states);
//}

//void LShapeTrackerUKF::lshapeToBoxModelConversion(double& x, double& y,double& vx, double& vy, double& L1, double& L2, double& theta, double& omega){
  //L1 = shape_kf.state()(0);
  //L2 = shape_kf.state()(1);
  //theta = shape_kf.state()(2);
  //omega = shape_kf.state()(3);
  ////Equations 30 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  //double ex = (L1 * cos(theta) + L2 * sin(theta)) /2;
  //double ey = (L1 * sin(theta) - L2 * cos(theta)) /2;
  //x = dynamic_kf.state()(0) + ex;
  //y = dynamic_kf.state()(1) + ey;

  ////Equations 31 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  ////TODO test the complete equation also
  //vx = dynamic_kf.state()(2);
  //vy = dynamic_kf.state()(3);

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
