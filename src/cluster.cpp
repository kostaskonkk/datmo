/**
 * Implementation of Cluster class.
*
* @author: Kostas Konstantinidis
* @date: 14.03.2019
*/
#include "cluster.hpp"

 
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
Cluster::Cluster(unsigned long int id, const pointList& new_points, const double& dt, const string& world_frame, const tf::Transform& ego_pose){

  this->id = id;
  this->r = rand() / double(RAND_MAX);
  this->g = rand() / double(RAND_MAX);
  this->b = rand() / double(RAND_MAX);
  a = 1.0;
  age = 1;
  red_flag = false;
  green_flag = false;
  blue_flag = false;
  frame_name = world_frame;

  new_cluster = new_points;

  ego_coordinates.first = ego_pose.getOrigin().getX();
  ego_coordinates.second= ego_pose.getOrigin().getY();

  // Initialization of Kalman Filter
  int n = 6;        // Number of states
  int m = 3;        // Number of measurements
  MatrixXd A(n, n); // System dynamics matrix
  MatrixXd C(m, n); // Output matrix
  MatrixXd Q(n, n); // Process noise covariance
  MatrixXd R(m, m); // Measurement noise covariance
  MatrixXd P(n, n); // Estimate error covariance
      
  A << 1, 0,dt, 0, 0, 0,
       0, 1, 0,dt, 0, 0,
       0, 0, 1, 0, 0, 0,
       0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 1,dt,
       0, 0, 0, 0, 0, 1;

  C << 1, 0, 0, 0, 0, 0,
       0, 1, 0, 0, 0, 0,
       0, 0, 0, 0, 1, 0;

  Q.setIdentity();
  Q *= 0.01;
  Q(2,2) = 1000;
  Q(3,3) = 1000;
  R.setIdentity();
  R *= 100;
  P.setIdentity();
  P *= 0.01;
  P(2,2) = 5;
  P(3,3) = 5;

  KalmanFilter kalman_filter(dt, A, C, Q, R, P); // Constructor for the filter
  this->kf_mean = kalman_filter;

  calcMean(new_points);
  previous_mean_values = mean_values;
  rectangleFitting(new_points);
  old_thetaL1 = thetaL1;
  old_thetaL2 = thetaL2;
  LShapeTracker l_shape_tracker(closest_corner_point, L1, L2, normalize_angle(thetaL1), dt);
  this->l_shape = l_shape_tracker;
  l_shape.lshapeToBoxModelConversion(cx, cy, cvx, cvy, L1_box, L2_box, th, comega);
  orientation = findOrientation(th, cvx, cvy);

  VectorXd x0(n);
  x0 << meanX(), meanY(), 0, 0, orientation, 0;
  kf_mean.init(0,x0);



  //Unscented Kalman Filter
  std::vector<double> args{0.001, 0, 2};
  //args[0] = 1;//alpha parameter
  //args[1] = 2;//kappa parameter
  //args[2] = 1;//beta parameter
  RobotLocalization::Ukf ukf_init(args);
  //this->ukf = ukf_init;

  int STATE_SIZE = 6;
  Eigen::MatrixXd initialCovar(STATE_SIZE, STATE_SIZE);
  initialCovar.setIdentity();
  initialCovar *= 1;
  initialCovar(3,3) *= 3;
  initialCovar(4,4) *= 3;
  initialCovar(5,5) *= 3;
  ukf_init.setEstimateErrorCovariance(initialCovar);

  Eigen::VectorXd initial_state(6);
  initial_state<<closest_corner_point.first,closest_corner_point.second,orientation,0,0,0;
  ukf_init.setState(initial_state);

  ukf_init.predict_ctrm(dt);


  //float ukf_beta  = 2;    //beta parameter
  LShapeTrackerUKF l_shape_tracker_ukf(ukf_init);
  this->l_shape_ukf = l_shape_tracker_ukf;

  populateTrackingMsgs(dt);
}

void Cluster::update(const pointList& new_points, const double dt, const tf::Transform& ego_pose) {

  ego_coordinates.first = ego_pose.getOrigin().getX();
  ego_coordinates.second= ego_pose.getOrigin().getY();

  age++;
  previous_mean_values = mean_values;
  new_cluster = new_points;

  calcMean(new_points);
  rectangleFitting(new_points);

  l_shape.detectCornerPointSwitch(old_thetaL1, thetaL1);
  
  double norm = normalize_angle(l_shape.shape_kf.state()(2));
  double distance = shortest_angular_distance(norm, thetaL1);
  double unwrapped_thetaL1 = distance + l_shape.shape_kf.state()(2) ;
  
  l_shape.update(closest_corner_point, L1, L2, unwrapped_thetaL1, dt);
  l_shape.lshapeToBoxModelConversion(cx, cy, cvx, cvy, L1_box, L2_box, th, comega);
  orientation = findOrientation(th, cvx, cvy);
  //ROS_INFO_STREAM("Orientation: "<<orientation);
  

  // Update Kalman Filter
  VectorXd y(3);

  y << meanX(), meanY(), orientation;
  kf_mean.update(y, dt);

  // UKF #######################
  Eigen::VectorXd measurement(6);

  measurement[0] = closest_corner_point.first;
  measurement[1] = closest_corner_point.second;
  measurement[2] = orientation;

  Eigen::MatrixXd measurementCovariance(6, 6);
  measurementCovariance.setIdentity();

  std::vector<int> updateVector(6, true);
  updateVector[3]=false;
  updateVector[4]=false;
  updateVector[5]=false;


  RobotLocalization::Measurement meas;
  meas.measurement_ = measurement;
  meas.covariance_ = measurementCovariance;
  meas.updateVector_ = updateVector;
  meas.mahalanobisThresh_ = std::numeric_limits<double>::max();

  l_shape_ukf.update(meas, dt);

  // UKF #######################

  populateTrackingMsgs(dt);

  //TODO Dynamic Static Classifier
  old_thetaL1 = thetaL1;
  old_thetaL2 = thetaL2;
}

void Cluster::populateTrackingMsgs(const double& dt){

    quaternion.setRPY(0,0,orientation);
    msg_track_mean.id = this->id;
    msg_track_mean.odom.header.stamp = ros::Time::now();
    msg_track_mean.odom.header.frame_id = frame_name;
    msg_track_mean.odom.pose.pose.position.x = meanX();
    msg_track_mean.odom.pose.pose.position.y = meanY();
    msg_track_mean.odom.pose.pose.orientation = tf2::toMsg(quaternion);
    msg_track_mean.length = length;
    msg_track_mean.width  = width;
    msg_track_mean.odom.twist.twist.linear.x = (mean_values.first- previous_mean_values.first)/dt;
    msg_track_mean.odom.twist.twist.linear.y = (mean_values.second- previous_mean_values.second)/dt;

    quaternion.setRPY(0,0,kf_mean.state()[4]);
    msg_track_mean_kf.id = this->id;
    msg_track_mean_kf.odom.header.stamp = ros::Time::now();
    msg_track_mean_kf.odom.header.frame_id = frame_name;
    msg_track_mean_kf.odom.pose.pose.position.x = kf_mean.state()[0];
    msg_track_mean_kf.odom.pose.pose.position.y = kf_mean.state()[1];
    msg_track_mean_kf.odom.twist.twist.linear.x = kf_mean.state()[2];
    msg_track_mean_kf.odom.twist.twist.linear.y = kf_mean.state()[3];
    msg_track_mean_kf.odom.pose.pose.orientation = tf2::toMsg(quaternion);
    msg_track_mean_kf.odom.twist.twist.angular.z =kf_mean.state()[5];
    msg_track_mean_kf.odom.pose.covariance[0] = kf_mean.P(0,0);
    msg_track_mean_kf.odom.pose.covariance[7] = kf_mean.P(1,1);
    msg_track_mean_kf.odom.twist.covariance[0]= kf_mean.P(2,2);
    msg_track_mean_kf.odom.twist.covariance[7]= kf_mean.P(3,3);

    msg_track_box_kf.id = this->id;
    msg_track_box_kf.odom.header.stamp = ros::Time::now();
    msg_track_box_kf.odom.header.frame_id = frame_name;
    msg_track_box_kf.odom.pose.pose.position.x = cx;
    msg_track_box_kf.odom.pose.pose.position.y = cy;
    msg_track_box_kf.odom.twist.twist.linear.x = cvx;
    msg_track_box_kf.odom.twist.twist.linear.y = cvy;
    msg_track_box_kf.odom.twist.twist.angular.z   =l_shape.shape_kf.state()[3];
    msg_track_box_kf.odom.pose.pose.orientation.z = th;
    msg_track_box_kf.length = L1_box;
    msg_track_box_kf.width  = L2_box;

    quaternion.setRPY(0,0, l_shape_ukf.ukf.getState()[2]);
    msg_track_box_ukf.id = this->id;
    msg_track_box_ukf.odom.header.stamp = ros::Time::now();
    msg_track_box_ukf.odom.header.frame_id = frame_name;
    msg_track_box_ukf.odom.pose.pose.position.x    = l_shape_ukf.ukf.getState()[0];
    msg_track_box_ukf.odom.pose.pose.position.y    = l_shape_ukf.ukf.getState()[1];
    msg_track_box_ukf.odom.pose.pose.orientation = tf2::toMsg(quaternion);
    msg_track_box_ukf.odom.twist.twist.linear.x    = l_shape_ukf.ukf.getState()[3];
    msg_track_box_ukf.odom.twist.twist.linear.y    = l_shape_ukf.ukf.getState()[4];
    msg_track_box_ukf.odom.twist.twist.angular.z   = l_shape_ukf.ukf.getState()[5];

}

void Cluster::rectangleFitting(const pointList& new_cluster){
  //This function is based on ¨Efficient L-Shape Fitting for
  //Vehicle Detection Using Laser Scanners¨
  auto begining = chrono::steady_clock::now(); //timing the execution

  unsigned int n = new_cluster.size();
  VectorXd e1(2),e2(2);
  MatrixXd X(n, 2); 
  for (unsigned int i = 0; i < n; ++i) {
    X(i,0) = new_cluster[i].first;
    X(i,1) = new_cluster[i].second;
  }
  VectorXd C1(n),C2(n);
  double q;
  unsigned int i =0;
  th = 0.0;
  //TODO make d configurable through Rviz
  unsigned int d = 40;
  ArrayX2d Q(d,2);
  float step = (3.14/2)/d;
  //#pragma omp parallel for
  for (i = 0; i < d; ++i) {
    e1 << cos(th), sin(th);
    e2 <<-sin(th), cos(th);
    C1 = X * e1;
    C2 = X * e2;

    //q = areaCriterion(C1,C2);
    q = closenessCriterion(C1,C2,0.001);
    Q(i,0) = th;
    Q(i,1) = q;

    th = th + step;
  }

  ArrayX2d::Index max_index;
  Q.col(1).maxCoeff(&max_index);//find Q with maximum value
  th = Q(max_index,0);
  e1 << cos(th), sin(th);
  e2 <<-sin(th), cos(th);
  C1 = X * e1;
  C2 = X * e2;
  // The coefficients of the four lines are calculated
  double a1,a2,a3,a4,b1,b2,b3,b4,c1,c2,c3,c4;
  a1 = cos(th);
  b1 = sin(th);
  c1 = C1.minCoeff();
  a2 = -sin(th);
  b2 = cos(th);
  c2 = C2.minCoeff();
  a3 = cos(th);
  b3 = sin(th);
  c3 = C1.maxCoeff();
  a4 = -sin(th);
  b4 = cos(th);
  c4 = C2.maxCoeff();

  vector<Point> corners;
  corners.push_back(lineIntersection(a2, b2, c2, a3, b3, c3));
  corners.push_back(lineIntersection(a1, b1, c1, a2, b2, c2));
  corners.push_back(lineIntersection(a1, b1, c1, a4, b4, c4));
  corners.push_back(lineIntersection(a4, b4, c4, a3, b3, c3));
  corner_list = corners;

  //Find the corner point that is closest to the ego vehicle
  double min_distance = pow(pow(corners[0].first - ego_coordinates.first,2.0)+pow(corners[0].second - ego_coordinates.second,2.0),0.5);
  unsigned int idx = 0;
  closest_corner_point = corners[0];
  double distance;
  for (unsigned int i = 1; i < 4; ++i) {
    distance = pow(pow(corners[i].first - ego_coordinates.first,2.0)+pow(corners[i].second - ego_coordinates.second,2.0),0.5);
    if (distance<min_distance) {
      min_distance = distance;  
      closest_corner_point = corners[i];
      idx = i;
    }
  }

  //Populate the l1l2 pointlist
  vector<Point> l1l2_list;
  if (idx==3) {
    l1l2_list.push_back(corners[0]);
  }
  else{
    l1l2_list.push_back(corners[idx+1]);
  }
  l1l2_list.push_back(corners[idx]);
  if (idx==0) {
    l1l2_list.push_back(corners[3]);
  } 
  else{
    l1l2_list.push_back(corners[idx-1]);
  }
  l1l2 = l1l2_list;


  double dx = l1l2_list[1].first - l1l2_list[0].first;
  double dy = l1l2_list[1].second- l1l2_list[0].second;
  L1 = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
  dx = l1l2_list[1].first - l1l2_list[2].first;
  dy = l1l2_list[1].second- l1l2_list[2].second;
  L2 = pow(pow(dx,2.0)+pow(dy,2.0),0.5);

  thetaL1   = atan2((l1l2[0].second - l1l2[1].second),(l1l2[0].first - l1l2[1].first)); 

  thetaL2 = atan2((l1l2[2].second - l1l2[1].second),(l1l2[2].first - l1l2[1].first)); 
  
  auto duration_nano = chrono::duration_cast<chrono::nanoseconds>(chrono::steady_clock::now() - begining);

  dur_size_rectangle_fitting.first = duration_nano.count();
  dur_size_rectangle_fitting.second = new_cluster.size();

} 
visualization_msgs::Marker Cluster::getBoundingBoxVisualisationMessage() {

  visualization_msgs::Marker bb_msg;
  bb_msg.header.stamp = ros::Time::now();
  bb_msg.header.frame_id  = frame_name;
  bb_msg.ns = "boundind_boxes";
  bb_msg.action = visualization_msgs::Marker::ADD;
  bb_msg.pose.orientation.w = 1.0;
  bb_msg.type = visualization_msgs::Marker::LINE_STRIP;
  bb_msg.id = this->id;
  bb_msg.scale.x = 0.03; //line width
  bb_msg.color.g = this->g;
  bb_msg.color.b = this->b;
  bb_msg.color.r = this->r;
  bb_msg.color.a = 1.0;

  geometry_msgs::Point p;
  for (unsigned int i = 0; i < 4; ++i) {
    p.x = corner_list[i].first;  
    p.y = corner_list[i].second;  
    bb_msg.points.push_back(p);
  }
  p.x = corner_list[0].first;  
  p.y = corner_list[0].second;  
  bb_msg.points.push_back(p);

  return bb_msg;
}
visualization_msgs::Marker Cluster::getBoxModelVisualisationMessage() {
  
  visualization_msgs::Marker bb_msg;

  bb_msg.header.stamp = ros::Time::now();
  bb_msg.header.frame_id  = frame_name;
  bb_msg.ns = "box_models";
  bb_msg.action = visualization_msgs::Marker::ADD;
  bb_msg.pose.orientation.w = 1.0;
  bb_msg.type = visualization_msgs::Marker::LINE_STRIP;
  bb_msg.id = this->id;
  bb_msg.scale.x = 0.05; //line width
  bb_msg.color.g = g;
  bb_msg.color.b = b;
  bb_msg.color.r = r;
  bb_msg.color.a = a;

  geometry_msgs::Point p;
  double x = L1_box/2;
  double y = L2_box/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = + L1_box/2;
  y = - L2_box/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = - L1_box/2;
  y = - L2_box/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = - L1_box/2;
  y = + L2_box/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = + L1_box/2;
  y = + L2_box/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  
  return bb_msg;
  
}
visualization_msgs::Marker Cluster::getLShapeVisualisationMessage() {

  visualization_msgs::Marker l1l2_msg;

  l1l2_msg.header.stamp = ros::Time::now();
  l1l2_msg.header.frame_id  = frame_name;
  l1l2_msg.ns = "L-Shapes";
  l1l2_msg.action = visualization_msgs::Marker::ADD;
  l1l2_msg.pose.orientation.w = 1.0;
  l1l2_msg.type = visualization_msgs::Marker::LINE_STRIP;
  l1l2_msg.id = this->id;
  l1l2_msg.scale.x = 0.1; //line width
  l1l2_msg.color.r = 0;
  l1l2_msg.color.g = 1;
  l1l2_msg.color.b = 0;
  l1l2_msg.color.a = 1.0;
  
  double theta_degrees = thetaL1 * (180.0/3.141592653589793238463);
  if (theta_degrees > 360){
    l1l2_msg.color.r = 1.0;
    l1l2_msg.color.g = 0;
    l1l2_msg.color.b = 0;
  }

  geometry_msgs::Point p;
  for (unsigned int i = 0; i < 3; ++i) {
    p.x = l1l2[i].first;
    p.y = l1l2[i].second;
    l1l2_msg.points.push_back(p);
  }

  return l1l2_msg;

}
Point Cluster::lineIntersection(double& a1, double& b1, double& c1, double& a2, double& b2, double& c2){
  // Function that returns the intersection point of two lines given their equations on
  // the form: a1x + b1x = c1, a2x + b2x = c2
  // There is no check for the determinant being zero because of the way the lines are 
  // calculated, it is not possible for this to happen.
  // source: geeksforgeeks point of intesection of two lines
  double determinant = a1*b2 - a2*b1;
  Point intersection_point;
  intersection_point.first  = (b2*c1 - b1*c2)/determinant;
  intersection_point.second = (a1*c2 - a2*c1)/determinant;

  return intersection_point;
}
double Cluster::areaCriterion(const VectorXd& C1, const VectorXd& C2){

  double a;
  a = -(C1.maxCoeff()-C1.minCoeff())*(C2.maxCoeff()-C2.minCoeff());
  
  return a; 

}
double Cluster::closenessCriterion(const VectorXd& C1, const VectorXd& C2, const float& d0){
  //Algorithm 4 of ¨Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners¨

  double c1_max, c1_min, c2_max, c2_min;
  c1_max = C1.maxCoeff();
  c1_min = C1.minCoeff();
  c2_max = C2.maxCoeff();
  c2_min = C2.minCoeff();
  VectorXd C1_max = c1_max - C1.array(); 
  VectorXd C1_min = C1.array() - c1_min;
  VectorXd D1, D2;
  if(C1_max.squaredNorm() < C1_min.squaredNorm()){
    D1 = C1_max;
  }
  else{
    D1 = C1_min;
  }
  VectorXd C2_max = c2_max - C2.array(); 
  VectorXd C2_min = C2.array() - c2_min;
  if(C2_max.squaredNorm() < C2_min.squaredNorm()){
    D2 = C2_max;
  }
  else{
    D2 = C2_min;
  }

  double d, min;
  double b =0 ;
  for (int i = 0; i < D1.size(); ++i) {
    if (D1(i) < D2(i)) {
      min = D1(i);
    }
    else{
      min = D2(i);
    } 
    if (min>d0) {
      d =  min;
    }
    else{
      d = d0;
    } 
    b = b + 1/d;
  }
 
  return b; 
}
double Cluster::findOrientation(const double& angle, const double& vx, const double& vy){

  vector<double> angles;
  double angle_norm = normalize_angle(angle);
  angles.push_back(angle_norm);
  angles.push_back(angle_norm + pi);
  angles.push_back(angle_norm + pi/2);
  angles.push_back(angle_norm + 3*pi/2);
  double vsp = tan(vy/vx);
  double min = 1.56;
  double distance;
  double orientation;
  for (unsigned int i = 0; i < 4; ++i) {
    distance = abs(shortest_angular_distance(vsp,angles[i]));
    if (distance < min){ 
      min = distance;
      orientation = normalize_angle(angles[i]);
      if(i<2){
        length = L1_box;
        width  = L2_box;
      }
      else{
        length = L2_box;
        width  = L1_box;
      }
    }
  } 
  
  return orientation;
  //ROS_INFO_STREAM("th_sp: "<<vsp<<", th: "<<th<<", th+pi/2: "<<th1<<", th+pi: "<<th2<<", th+3pi/2: "<<th3);
  //ROS_INFO_STREAM("th_sp: "<<vsp<<", orientation: "<<orientation);
  
}
visualization_msgs::Marker Cluster::getThetaBoxVisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "thetaBox";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.color.a = 1.0;
  arrow_marker.color.g = this->g;
  arrow_marker.color.b = this->b;
  arrow_marker.color.r = this->g;
  arrow_marker.id = this->id;
  arrow_marker.pose.position.x = cx;
  arrow_marker.pose.position.y = cy;
  arrow_marker.pose.position.z = 0;
  arrow_marker.scale.x = 0.8;
  arrow_marker.scale.y = 0.3;  
  arrow_marker.scale.z = 0.1;  

  arrow_marker.header.frame_id = frame_name;

  quaternion.setRPY(0,0,orientation);
  arrow_marker.pose.orientation = tf2::toMsg(quaternion);
 
  return arrow_marker;
}
visualization_msgs::Marker Cluster::getThetaL1VisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "thetaL1";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.color.a = 1.0;
  arrow_marker.color.g = 0;
  arrow_marker.color.b = 0;
  arrow_marker.color.r = 1;
  arrow_marker.id = this->id;

  arrow_marker.header.frame_id = frame_name;
  tf2::Quaternion quat_theta;
  quat_theta.setRPY(0,0,thetaL1);
  arrow_marker.pose.orientation = tf2::toMsg(quat_theta);
  arrow_marker.pose.position.x = closest_corner_point.first;
  arrow_marker.pose.position.y = closest_corner_point.second;
  arrow_marker.pose.position.z = 0;
  arrow_marker.scale.x = 0.2;
  arrow_marker.scale.y = 0.1;  
  arrow_marker.scale.z = 0.001;  
 
  return arrow_marker;
}
visualization_msgs::Marker Cluster::getThetaL2VisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  //arrow_marker.header.frame_id = frame_name;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "thetaL2";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.color.a = 1.0;
  arrow_marker.color.g = 1;
  arrow_marker.color.b = 0;
  arrow_marker.color.r = 0;
  arrow_marker.id = this->id;

  arrow_marker.header.frame_id = frame_name;
  tf2::Quaternion quat_theta;
  quat_theta.setRPY(0,0,thetaL2);
  //quat_theta.normalize();
  arrow_marker.pose.orientation = tf2::toMsg(quat_theta);
  arrow_marker.pose.position.x = closest_corner_point.first;
  arrow_marker.pose.position.y = closest_corner_point.second;
  arrow_marker.scale.x = 0.2;
  arrow_marker.scale.y = 0.1;  
  arrow_marker.scale.z = 0.01;  
  return arrow_marker;
}
visualization_msgs::Marker Cluster::getPoseCovariance(){

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_name;
    marker.header.stamp = ros::Time::now();
    marker.ns = "covariances";
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.r = r;
    marker.id = this->id;

    marker.pose.position.x = kf_mean.state()[0];
    marker.pose.position.y = kf_mean.state()[1];
    Eigen::Matrix2f covMatrix(2,2);
    covMatrix << kf_mean.P(0),kf_mean.P(1),
                 kf_mean.P(6),kf_mean.P(7);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

    const Eigen::Vector2f& eigValues (eig.eigenvalues());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

    float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

    marker.type = visualization_msgs::Marker::CYLINDER;

    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);

    marker.scale.x = lengthMajor;
    marker.scale.y = lengthMinor;
    marker.scale.z = 0.001;

    marker.pose.orientation.w = cos(angle*0.5);
    marker.pose.orientation.z = sin(angle*0.5);

    return marker;

  }
visualization_msgs::Marker Cluster::getArrowVisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  //arrow_marker.header.frame_id = frame_name;
  arrow_marker.header.frame_id = frame_name;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "velocities";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  arrow_marker.color.a = 1.0;
  arrow_marker.color.g = g;
  arrow_marker.color.b = b;
  arrow_marker.color.r = r;
  arrow_marker.id = this->id;
  arrow_marker.scale.x = 0.1;    //Shaft diameter of the arrow
  arrow_marker.scale.y = 0.2;    //Head  diameter of the arrow

  geometry_msgs::Point p;
  p.x = cx; 
  p.y = cy; 
  p.z = 0;
  arrow_marker.points.push_back(p);

  p.x = cx+ cvx *1.5; 
  p.y = cy+ cvy *1.5; 
  p.z = 0;
  arrow_marker.points.push_back(p);
  return arrow_marker;
}
 visualization_msgs::Marker Cluster::getClosestCornerPointVisualisationMessage() {

  visualization_msgs::Marker corner_msg;
  corner_msg.type = visualization_msgs::Marker::POINTS;
  corner_msg.header.frame_id = frame_name;
  corner_msg.header.stamp = ros::Time::now();
  corner_msg.ns = "closest_corner";
  corner_msg.action = visualization_msgs::Marker::ADD;
  corner_msg.pose.orientation.w = 1.0;    
  corner_msg.scale.x = 0.1;
  corner_msg.scale.y = 0.1;  
  corner_msg.color.a = 1.0;
  corner_msg.color.g = 0.0;
  corner_msg.color.b = 0.0;
  corner_msg.color.r = 0.0;
  corner_msg.id = this->id;

  geometry_msgs::Point p;
  p.x = closest_corner_point.first; 
  p.y = closest_corner_point.second;
  p.z = 0;
  corner_msg.points.push_back(p);

  return corner_msg;
}
 visualization_msgs::Marker Cluster::getBoundingBoxCenterVisualisationMessage() {

    visualization_msgs::Marker boxcenter_marker;
    boxcenter_marker.type = visualization_msgs::Marker::POINTS;
    boxcenter_marker.header.frame_id = frame_name;
    boxcenter_marker.header.stamp = ros::Time::now();
    boxcenter_marker.ns = "bounding_box_center";
    boxcenter_marker.action = visualization_msgs::Marker::ADD;
    boxcenter_marker.pose.orientation.w = 1.0;    
    boxcenter_marker.scale.x = 0.1;
    boxcenter_marker.scale.y = 0.1;  
    boxcenter_marker.color.a = 1.0;
    boxcenter_marker.color.r = 1;
    boxcenter_marker.color.g = 1;
    boxcenter_marker.color.b = 0;
    boxcenter_marker.id = this->id;
    
    geometry_msgs::Point p;
    //p.x = cx; 
    //p.y = cy;
    //ROS_WARN_STREAM("State is:\n"<<ukf.getState()<<"\n");
   
    p.x = ukf.getState()[0];
    p.y = ukf.getState()[1];
    //p.y = ukf.StateMemberY; 
    boxcenter_marker.points.push_back(p);

  return boxcenter_marker;
}
visualization_msgs::Marker Cluster::getClusterVisualisationMessage() {

  visualization_msgs::Marker cluster_vmsg;
  cluster_vmsg.header.frame_id  = frame_name;
  cluster_vmsg.header.stamp = ros::Time::now();
  cluster_vmsg.ns = "clusters";
  cluster_vmsg.action = visualization_msgs::Marker::ADD;
  cluster_vmsg.pose.orientation.w = 1.0;
  cluster_vmsg.type = visualization_msgs::Marker::POINTS;
  cluster_vmsg.scale.x = 0.08;
  cluster_vmsg.scale.y = 0.08;
  //cluster_vmsg.lifetime = ros::Duration(0.09);
  cluster_vmsg.id = this->id;

  cluster_vmsg.color.g = this->g;
  cluster_vmsg.color.b = this->b;
  cluster_vmsg.color.r = this->r;
  cluster_vmsg.color.a = 1.0;


  geometry_msgs::Point p;
 
  for(unsigned int j=0; j<new_cluster.size(); ++j){
    p.x = new_cluster[j].first;
    p.y = new_cluster[j].second;
    p.z = 0;
    cluster_vmsg.points.push_back(p);
  }

  return cluster_vmsg;
}
visualization_msgs::Marker Cluster::getLineVisualisationMessage() {

  visualization_msgs::Marker line_msg;


  line_msg.header.stamp = ros::Time::now();
  line_msg.header.frame_id  = frame_name;
  line_msg.ns = "lines";
  line_msg.action = visualization_msgs::Marker::ADD;
  line_msg.pose.orientation.w = 1.0;
  line_msg.type = visualization_msgs::Marker::LINE_STRIP;
  line_msg.id = this->id;
  line_msg.scale.x = 0.1; //line width
  line_msg.lifetime = ros::Duration(0.09);
  line_msg.color.g = this->g;
  line_msg.color.b = this->b;
  line_msg.color.r = this->r;
  line_msg.color.a = 1.0;


  //Feed the cluster into the Iterative End-Point Fit Function
  //and the l_shape_extractor and then save them into the l_shapes vector
  // Line and L-Shape Extraction


  vector<Point> pointListOut;
  Cluster::ramerDouglasPeucker(new_cluster, 0.1, pointListOut);
  geometry_msgs::Point p;
  for(unsigned int k =0 ;k<pointListOut.size();++k){
    p.x = pointListOut[k].first;
    p.y = pointListOut[k].second;
    p.z = 0;

    line_msg.points.push_back(p);
  }
  double l;
  if(pointListOut.size()==1){
  for(unsigned int k=0; k<pointListOut.size()-1;++k){
    l = sqrt(pow(pointListOut[k+1].first - pointListOut[k].first,2) + pow(pointListOut[k+1].second - pointListOut[k].second,2));
  }
  }
  return line_msg;

}
void Cluster::calcMean(const pointList& c){

  double sum_x = 0, sum_y = 0;

  for(unsigned int i = 0; i<c.size(); ++i){

    sum_x = sum_x + c[i].first;
    sum_y = sum_y + c[i].second;
  }

    this->mean_values.first = sum_x / c.size();
    this->mean_values.second= sum_y / c.size();
}
double Cluster::perpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd){
  //2D implementation of the Ramer-Douglas-Peucker algorithm
  //By Tim Sheerman-Chase, 2016
  //Released under CC0
  //https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
  double dx = lineEnd.first - lineStart.first;
  double dy = lineEnd.second - lineStart.second;

  //Normalise
  double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
  if(mag > 0.0)
  {
    dx /= mag; dy /= mag;
  }

  double pvx = pt.first - lineStart.first;
  double pvy = pt.second - lineStart.second;

  //Get dot product (project pv onto normalized direction)
  double pvdot = dx * pvx + dy * pvy;

  //Scale line direction vector
  double dsx = pvdot * dx;
  double dsy = pvdot * dy;

  //Subtract this from pv
  double ax = pvx - dsx;
  double ay = pvy - dsy;

  return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}
void Cluster::ramerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out){
  //2D implementation of the Ramer-Douglas-Peucker algorithm
  //By Tim Sheerman-Chase, 2016
  //Released under CC0
  //https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm

  // Find the point with the maximum distance from line between start and end
  double dmax = 0.0;
  size_t index = 0;
  size_t end = pointList.size()-1;
  for(size_t i = 1; i < end; i++)
  {
    double d = perpendicularDistance(pointList[i], pointList[0], pointList[end]);
    if (d > dmax)
    {
      index = i;
      dmax = d;
    }
  }

  // If max distance is greater than epsilon, recursively simplify
  if(dmax > epsilon)
  {
    // Recursive call
    vector<Point> recResults1;
    vector<Point> recResults2;
    vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
    vector<Point> lastLine(pointList.begin()+index, pointList.end());
    ramerDouglasPeucker(firstLine, epsilon, recResults1);
    ramerDouglasPeucker(lastLine, epsilon, recResults2);
 
    // Build the result list
    out.assign(recResults1.begin(), recResults1.end()-1);
    out.insert(out.end(), recResults2.begin(), recResults2.end());
    if(out.size()<2)
      throw runtime_error("Problem assembling output");
  } 
  else 
  {
    //Just return start and end points
    out.clear();
    out.push_back(pointList[0]);
    out.push_back(pointList[end]);
  }
}
