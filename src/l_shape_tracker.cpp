/*
 * Copyright (c) 2020, Robobrain.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Konstantinos Konstantinidis */

#include "l_shape_tracker.hpp"

static inline double normalize_angle_positive(double angle){
  //Normalizes the angle to be 0 to 2*M_PI.
  //It takes and returns radians.
  return fmod(fmod(angle, 2.0*M_PI) + 2.0*M_PI, 2.0*M_PI);
}
static inline double normalize_angle(double angle){
 //Normalizes the angle to be -M_PI circle to +M_PI circle
 //It takes and returns radians.
  double a = normalize_angle_positive(angle);
  if (a > M_PI)
    a -= 2.0 *M_PI;
  return a;
}

static inline double shortest_angular_distance(double from, double to){
  //Given 2 angles, this returns the shortest angular difference.
  //The inputs and outputs are radians.
  //The result would always be -pi <= result <= pi.
  //Adding the result to "from" results to "to".
  return normalize_angle(to-from);
}

LshapeTracker::LshapeTracker(){}//Creates a blank estimator

LshapeTracker::LshapeTracker(const double& x_corner, const double& y_corner, const double& L1, const double& L2, const double& theta, const double& dt){

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
  R.setIdentity();
  R *=10;
  R *= 0.1;
  P.setIdentity() * 0.1;

  KalmanFilter dynamic_kalman_filter(dt, A, C, Q, R, P); 
  this->dynamic_kf = dynamic_kalman_filter;

  VectorXd x0_dynamic(n);
  x0_dynamic << x_corner, y_corner, 0, 0, 0, 0;
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

  Qs<< 0.04, 0,   0, 0,
       0, 0.04,  0, 0,
       0,    0, dt, pow(dt,2)/2,
       0,    0,  0,dt;
  Ps.setIdentity();

  KalmanFilter shape_kalman_filter(dt, As, Cs, Qs, Rs, Ps); 
  this->shape_kf = shape_kalman_filter;

  VectorXd x0_shape(n);
  double L1init=0.1;
  double L2init=0.1;
  if(L1>L1init){L1init = L1;}
  if(L2>L2init){L2init = L2;}
  x0_shape << L1init, L2init, theta, 0;
  shape_kf.init(0,x0_shape);


  L1_old = L1;
  L2_old = L2;
  old_thetaL1 = theta;
  x_old = x_corner;
  y_old = y_corner;

}

void LshapeTracker::update(const double& thetaL1, const double& x_corner, const double& y_corner, const double& L1, const double& L2, const double& dt, const int cluster_size) {

  current_size = cluster_size;
  detectCornerPointSwitch(old_thetaL1, thetaL1, dt);

  double norm = normalize_angle(shape_kf.state()(2));
  double distance = shortest_angular_distance(norm, thetaL1);
  double theta = distance + shape_kf.state()(2) ;
  
  // Update Dynamic Kalman Filter
  Vector2d y;
  y << x_corner, y_corner;
  dynamic_kf.update(y, dt);

  // Update Shape Kalman Filter
  Vector3d shape_measurements;
  double L1max, L2max;
  L2max = L2;
  L1max = L1;
  shape_kf.R<< pow(L1,-2), 0, 0,
               0, pow(L2,-2), 0,
               0,      0, 0.5;
  shape_measurements << L1max, L2max, theta;
  shape_kf.update(shape_measurements, dt);

  L1_old = L1;
  L2_old = L2;
  old_thetaL1 = thetaL1;
  x_old = x_corner;
  y_old = y_corner;

}

void LshapeTracker::detectCornerPointSwitchMahalanobis(const double& from, const double& to, const double L1, const double L2, const double x_corner, const double y_corner){
  // The purpose of this function is to detect potential corner point switches.
  // For this purpose it calculates the Mahalanobis distance between the previous
  // and the current measurements and based on the lowest distance it decides if 
  // the corner point changed or not.
  
  double x_new = x_corner;
  double y_new = y_corner;
  double theta_new = to;
  double theta_corner = from;
  double x_c = dynamic_kf.state()(0);
  double y_c = dynamic_kf.state()(1);
  double L1_box = shape_kf.state()(0);
  double L2_box = shape_kf.state()(1);

  double x_corner_L1= x_c + L1_box*cos(theta_corner);
  double y_corner_L1= y_c + L1_box*sin(theta_corner);
  double theta_corner_L1 = normalize_angle(theta_corner + pi/2);

  double x_corner_L2 = x_c + L2_box*sin(theta_corner);
  double y_corner_L2 = y_c - L2_box*cos(theta_corner);
  double theta_corner_L2 = normalize_angle(theta_corner + pi/2);
  ROS_DEBUG_STREAM("simple: "<<theta_corner-theta_new<<", findTurn: "<<findTurn(theta_new,theta_corner));

   Eigen::Matrix<double, 5, 5> C;
   C.setZero();
   C(0,0) = dynamic_kf.P(0,0);
   C(0,1) = dynamic_kf.P(0,1);
   C(1,0) = dynamic_kf.P(1,0);
   C(1,1) = dynamic_kf.P(1,1);
   C(2,2) = shape_kf.P(0,0);
   C(2,3) = shape_kf.P(0,1);
   C(3,2) = shape_kf.P(1,0);
   C(3,3) = shape_kf.P(1,1);
   C(4,4) = shape_kf.P(2,2);

   Eigen::Matrix<double, 5, 1> means;
   means(0) = x_c - x_new;
   means(1) = y_c - y_new;
   means(2) = L1-L1_box;
   means(3) = L2-L2_box;
   means(4) = findTurn(theta_new,theta_corner);
   std::vector<double> mdistances;
   mdistances.push_back(means.transpose()*C.inverse()*means);

   means(0) = x_corner_L1 - x_new;
   means(1) = y_corner_L1 - y_new;
   means(2) = L1-L2_box;
   means(3) = L2-L1_box;
   means(4) = findTurn(theta_new,theta_corner_L1);
   mdistances.push_back(means.transpose()*C.inverse()*means);

   means(0) = x_corner_L2 - x_new;
   means(1) = y_corner_L2 - y_new;
   means(2) = L1-L2_box;
   means(3) = L2-L1_box;
   means(4) = findTurn(theta_new,theta_corner_L2);
   mdistances.push_back(means.transpose()*C.inverse()*means);

  int minElementIndex = std::min_element(mdistances.begin(),mdistances.end()) - mdistances.begin();

  if(minElementIndex == 2 && abs(mdistances[0]-mdistances[2])>0.1 && current_size>1){
   this->CounterClockwisePointSwitch();
  }
  else if(minElementIndex == 1 && abs(mdistances[0]-mdistances[1])>0.1 && current_size>1){
   this->ClockwisePointSwitch();
  }


  std::vector<double> distances; 
  double euclidean;
  euclidean = sqrt(pow(x_c-x_new,2) + pow(y_c-y_new,2) + pow(findTurn(theta_new,theta_corner),2) + pow(L1-L1_box,2) + pow(L2-L2_box,2));
  distances.push_back(euclidean);
  euclidean = sqrt(pow(x_corner_L1-x_new,2) + pow(y_corner_L1-y_new,2) + pow(findTurn(theta_new,theta_corner_L1),2)+ pow(L1-L2_box,2) + pow(L2-L1_box,2));
  distances.push_back(euclidean);
  euclidean = sqrt(pow(x_corner_L2-x_new,2) + pow(y_corner_L2-y_new,2) + pow(findTurn(theta_new,theta_corner_L2),2)+ pow(L1-L2_box,2) + pow(L2-L1_box,2));
  distances.push_back(euclidean);

}

  void LshapeTracker::BoxModel(double& x, double& y,double& vx, double& vy,double& theta, double& psi, double& omega, double& L1, double& L2, double& length, double& width){
  L1 = shape_kf.state()(0);
  L2 = shape_kf.state()(1);
  theta = shape_kf.state()(2);
  //Equations 30 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  double ex = (L1 * cos(theta) + L2 * sin(theta)) /2;
  double ey = (L1 * sin(theta) - L2 * cos(theta)) /2;

  omega = shape_kf.state()(3);
  x = dynamic_kf.state()(0) + ex;
  y = dynamic_kf.state()(1) + ey;

  //Equations 31 of "L-Shape Model Switching-Based precise motion tracking of moving vehicles"
  //TODO test the complete equation also
  vx = dynamic_kf.state()(2);
  vy = dynamic_kf.state()(3);

  findOrientation(psi, length, width);
}


double LshapeTracker::findTurn(const double& new_angle, const double& old_angle){
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

void LshapeTracker::detectCornerPointSwitch(const double& from, const double& to, const double dt){
  //Corner Point Switch Detection
  double turn = findTurn(from, to);
    if(turn <-0.8){
     this->CounterClockwisePointSwitch();
    }
    else if(turn > 0.6){
     this->ClockwisePointSwitch();
    }
}

void LshapeTracker::findOrientation(double& psi, double& length, double& width){
  //This function finds the orientation of a moving object, when given an L-shape orientation

  std::vector<double> angles;
  double angle_norm = normalize_angle(shape_kf.state()(2));
  angles.push_back(angle_norm);
  angles.push_back(angle_norm + pi);
  angles.push_back(angle_norm + pi/2);
  angles.push_back(angle_norm + 3*pi/2);

  double vsp = atan2(dynamic_kf.state()(3),dynamic_kf.state()(2));
  double min = 1.56;
  double distance;
  double orientation;
  int    pos;
  for (unsigned int i = 0; i < 4; ++i) {
    distance = abs(shortest_angular_distance(vsp,angles[i]));
    if (distance < min){ 
      min = distance;
      orientation = normalize_angle(angles[i]);
      pos = i;
    }
  } 
  if(pos ==0 || pos==1){
    length = shape_kf.state()(0);
    width  = shape_kf.state()(1);
  }
  else{
    length = shape_kf.state()(1);
    width  = shape_kf.state()(0);
  }

  psi = normalize_angle(orientation);
  
}


void LshapeTracker::ClockwisePointSwitch(){
  // Equation 17

  Vector6d new_dynamic_states = dynamic_kf.state();
  Vector4d new_shape_states = shape_kf.state();

  double L1 = shape_kf.state()(0);
  double L2 = shape_kf.state()(1);

  //x = x + L1 * cos(theta);
  new_dynamic_states(0) += L1 * cos(shape_kf.state()(2));
  //y = y + L1 * sin(theta);
  new_dynamic_states(1) += L1 * sin(shape_kf.state()(2));
  //vx = vx - L1 * omega * sin(theta);
  new_dynamic_states(2) -= L1 * shape_kf.state()(3) *  sin(shape_kf.state()(2));
  //vy = vy + L1 * omega * cos(theta);
  new_dynamic_states(3) += L1 * shape_kf.state()(3) *  cos(shape_kf.state()(2));
  //ax = ax - L1 * omega^2 * cos(theta);
  new_dynamic_states(4) -= L1 * pow(shape_kf.state()(3),2) *  cos(shape_kf.state()(2));
  //ay = ay - L1 * omega^2 * sin(theta);
  new_dynamic_states(5) -= L1 * pow(shape_kf.state()(3),2) *  sin(shape_kf.state()(2));


  //L1 = L2
  new_shape_states(0) = L2;
  //L2 = L1
  new_shape_states(1) = L1;

  new_shape_states(2) = shape_kf.state()(2) - pi / 2;

  dynamic_kf.changeStates(new_dynamic_states);
  shape_kf.changeStates(new_shape_states);

}

void LshapeTracker::CounterClockwisePointSwitch(){
  // Equation 17

  Vector6d new_dynamic_states = dynamic_kf.state();
  Vector4d new_shape_states = shape_kf.state();

  double L1 = shape_kf.state()(0);
  double L2 = shape_kf.state()(1);

  //x = x + L2 * sin(theta);
  new_dynamic_states(0) += L2 * sin(shape_kf.state()(2));
  //y = y - L2 * cos(theta);
  new_dynamic_states(1) -= L2 * cos(shape_kf.state()(2));
  //vx = vx + L2 * omega * cos(theta);
  new_dynamic_states(2) += L2 * shape_kf.state()(3) *  cos(shape_kf.state()(2));
  //vy = vy + L2 * omega * sin(theta);
  new_dynamic_states(3) +=  L2 * shape_kf.state()(3) *  sin(shape_kf.state()(2));
  //ax = ax - L2 * omega^2 * cos(theta);
  new_dynamic_states(4) = dynamic_kf.state()(4) - L2 * pow(shape_kf.state()(3),2) *  sin(shape_kf.state()(2));
  //ay = ay - L2 * omega^2 * sin(theta);
  new_dynamic_states(5) = dynamic_kf.state()(5) + L2 * pow(shape_kf.state()(3),2) *  cos(shape_kf.state()(2));

  //L1 = L2
  new_shape_states(0) = L2;
  //L2 = L1
  new_shape_states(1) = L1;

  new_shape_states(2) = shape_kf.state()(2) + pi / 2;

  dynamic_kf.changeStates(new_dynamic_states);
  shape_kf.changeStates(new_shape_states);
}


