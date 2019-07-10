/**
 * Implementation of Cluster class.
*
* @author: Kostas Konstantinidis
* @date: 14.03.2019
*/

#include "cluster.hpp"


Cluster::Cluster(unsigned long int id, const pointList& new_points, const double& dt, const tf::TransformListener& tf_listener, const string& source_frame,const string& target_frame){

  this->id = id;
  this->r = rand() / double(RAND_MAX);
  this->g = rand() / double(RAND_MAX);
  this->b = rand() / double(RAND_MAX);
  this->moving = true; //all clusters at the beginning are initialised as moving
  age = 1;
  p_source_frame_name_ = source_frame;
  p_target_frame_name_ = target_frame;

  new_cluster = new_points;

  // Initialization of Kalman Filter
  int n = 4; // Number of states
  int m = 4; // Number of measurements
  // double dt = 0.1;  // Time step
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
       0, 1, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1;

  Q.setIdentity();
  R.setIdentity();
  P.setIdentity();

  KalmanFilter kalman_filter(dt, A, C, Q, R, P); // Constructor for the filter
  this->kf = kalman_filter;
  this->map_kf = kalman_filter;

  calcMean(new_points);
  rectangleFitting(new_points);
  LShapeTracker tr(this->closest_corner_point, this->L1, this->L2, this->theta, dt);
  this->tracker = tr;

  VectorXd x0(n);
  x0 << Cluster::meanX(), Cluster::meanY(), 0, 0;
  kf.init(0,x0);


  if(tf_listener.canTransform(p_target_frame_name_, p_source_frame_name_, ros::Time())){

    pose_source_.header.stamp = ros::Time(0);
    pose_source_.pose.orientation.w = 1.0;
    pose_source_.header.frame_id = p_source_frame_name_;
    pose_source_.pose.position.x = meanX();
    pose_source_.pose.position.y = meanY();
    pose_source_.pose.orientation.w = 1; 

    geometry_msgs::PoseStamped pose_out;

    tf_listener.transformPose(p_target_frame_name_, pose_source_, pose_out);

    abs_mean_values.first = pose_out.pose.position.x;
    abs_mean_values.second = pose_out.pose.position.y;
    abs_previous_mean_values = abs_mean_values;

    trajectory_.header.stamp = pose_out.header.stamp;
    trajectory_.header.frame_id = pose_out.header.frame_id;
    trajectory_.poses.push_back(pose_out);
    
    track_msg.id = this->id;
    track_msg.odom.header.stamp = pose_out.header.stamp;
    track_msg.odom.header.frame_id = pose_out.header.frame_id;
    track_msg.odom.pose.pose = pose_out.pose;

    //Initialise Kalman filter on Map coordinates
    x0 << pose_out.pose.position.x, pose_out.pose.position.y, 0, 0;
    map_kf.init(0,x0);
  }

  else{ //If the tf is not possible init all states at 0
    x0 << 0, 0, 0, 0;
    map_kf.init(0,x0);
  };

  //Populate filtered track msg
  filtered_track_msg.id = this->id;
  filtered_track_msg.odom.header.stamp = ros::Time::now();
  filtered_track_msg.odom.header.frame_id = p_target_frame_name_;
  filtered_track_msg.odom.pose.pose.position.x = map_kf.state()[0];
  filtered_track_msg.odom.pose.pose.position.y = map_kf.state()[1];
  filtered_track_msg.odom.twist.twist.linear.x = map_kf.state()[2];
  filtered_track_msg.odom.twist.twist.linear.y = map_kf.state()[3];

  
}

void Cluster::update(const pointList& new_points, const double dt_in, const tf::TransformListener& tf_listener) {

  age++;
  previous_mean_values = mean_values;
  abs_previous_mean_values = abs_mean_values;
  new_cluster = new_points;

  Cluster::calcMean(new_points);
  rectangleFitting(new_points);
  //Corner Point Switch Detection
  if(theta < tracker.shape.state()(2) - 1.2){
    tracker.ClockwisePointSwitch();
  }
  else if(theta > tracker.shape.state()(2) + 1.2){
    tracker.CounterClockwisePointSwitch();
  }
  tracker.update(this->closest_corner_point, this->L1, this->L2, this->theta, dt_in);
  this->dt = dt_in;

  // Update Kalman Filter
  VectorXd y(4);
  double vx, vy;
  vx = (mean_values.first - previous_mean_values.first)/dt;
  vy = (mean_values.second - previous_mean_values.second)/dt;

  y << meanX(), meanY(), vx, vy;
  kf.update(y, dt);

  if(tf_listener.canTransform(p_target_frame_name_, p_source_frame_name_, ros::Time())){

    pose_source_.header.stamp = ros::Time(0);
    pose_source_.pose.orientation.w = 1.0;
    pose_source_.header.frame_id = p_source_frame_name_;
    pose_source_.pose.position.x = meanX();
    pose_source_.pose.position.y = meanY();
    pose_source_.pose.orientation.w = 1; 

    geometry_msgs::PoseStamped pose_out;

    tf_listener.transformPose(p_target_frame_name_, pose_source_, pose_out);

    abs_mean_values.first = pose_out.pose.position.x;
    abs_mean_values.second = pose_out.pose.position.y;

    trajectory_.header.stamp = pose_out.header.stamp;
    trajectory_.header.frame_id = pose_out.header.frame_id;
    trajectory_.poses.push_back(pose_out);
    
    track_msg.id = this->id;
    track_msg.odom.header.stamp = pose_out.header.stamp;
    track_msg.odom.header.frame_id = pose_out.header.frame_id;
    track_msg.odom.pose.pose = pose_out.pose;

    //Update Kalman filter on Map coordinates
    //double avx, avy;
    avx = (abs_mean_values.first - abs_previous_mean_values.first)/dt;
    avy = (abs_mean_values.second - abs_previous_mean_values.second)/dt;
    y << pose_out.pose.position.x, pose_out.pose.position.y, avx, avy;
    map_kf.update(y, dt);

    //Populate filtered track msg
    filtered_track_msg.id = this->id;
    filtered_track_msg.odom.header.stamp = ros::Time::now();
    filtered_track_msg.odom.header.frame_id = p_target_frame_name_;
    filtered_track_msg.odom.pose.pose.position.x = map_kf.state()[0];
    filtered_track_msg.odom.pose.pose.position.y = map_kf.state()[1];
    filtered_track_msg.odom.twist.twist.linear.x = map_kf.state()[2];
    filtered_track_msg.odom.twist.twist.linear.y = map_kf.state()[3];

    //TODO Dynamic Static Classifier
  } 
}

void Cluster::rectangleFitting(const pointList& new_cluster){
  //This function is based on ¨Efficient L-Shape Fitting for
  //Vehicle Detection Using Laser Scanners¨
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
  double th = 0.0;
  //TODO make d configurable through Rviz
  unsigned int d = 100;
  ArrayX2d Q(d,2);
  float step = (3.14/2)/d;
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
  double min_distance = pow(pow(corners[0].first,2.0)+pow(corners[0].second,2.0),0.5);
  unsigned int idx = 0;
  closest_corner_point = corners[0];
  double distance;
  for (unsigned int i = 1; i < 4; ++i) {
    distance = pow(pow(corners[i].first,2.0)+pow(corners[i].second,2.0),0.5);
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

  theta = atan2((l1l2[0].second - l1l2[1].second),(l1l2[0].first - l1l2[1].first)); 


} 
 
visualization_msgs::Marker Cluster::getBoundingBoxVisualisationMessage() {

  visualization_msgs::Marker bb_msg;
  bb_msg.header.stamp = ros::Time::now();
  bb_msg.header.frame_id  = p_source_frame_name_;
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
  //if(!moving){return bb_msg;};//cluster not moving-empty msg

  bb_msg.header.stamp = ros::Time::now();
  bb_msg.header.frame_id  = p_source_frame_name_;
  bb_msg.ns = "box_models";
  bb_msg.action = visualization_msgs::Marker::ADD;
  bb_msg.pose.orientation.w = 1.0;
  bb_msg.type = visualization_msgs::Marker::LINE_STRIP;
  bb_msg.id = this->id;
  bb_msg.scale.x = 0.05; //line width
  bb_msg.color.g = this->g;
  bb_msg.color.b = this->b;
  bb_msg.color.r = this->r;
  bb_msg.color.a = 1.0;
  
  double cx, cy, th, L1, L2; 
  tracker.lshapeToBoxModelConversion(cx, cy, L1, L2, th);

  geometry_msgs::Point p;
  double x = L1/2;
  double y = L2/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = + L1/2;
  y = - L2/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = - L1/2;
  y = - L2/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = - L1/2;
  y = + L2/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  x = + L1/2;
  y = + L2/2;
  p.x = cx + x*cos(th) - y*sin(th);
  p.y = cy + x*sin(th) + y*cos(th);
  bb_msg.points.push_back(p);
  
  return bb_msg;
  //double cx = map_kf.state()[0]; 
  //double cy = map_kf.state()[1]; 
  //double th = 0;
  //double width = 0.3;
  //double length = 0.6;

  //geometry_msgs::Point p;
  //double x = width/2;
  //double y = length/2;
  //p.x = cx + x*cos(th) - y*sin(th);
  //p.y = cy + x*sin(th) + y*cos(th);
  //bb_msg.points.push_back(p);
  //x = + width/2;
  //y = - length/2;
  //p.x = cx + x*cos(th) - y*sin(th);
  //p.y = cy + x*sin(th) + y*cos(th);
  //bb_msg.points.push_back(p);
  //x = - width/2;
  //y = - length/2;
  //p.x = cx + x*cos(th) - y*sin(th);
  //p.y = cy + x*sin(th) + y*cos(th);
  //bb_msg.points.push_back(p);
  //x = - width/2;
  //y = + length/2;
  //p.x = cx + x*cos(th) - y*sin(th);
  //p.y = cy + x*sin(th) + y*cos(th);
  //bb_msg.points.push_back(p);
  //x = + width/2;
  //y = + length/2;
  //p.x = cx + x*cos(th) - y*sin(th);
  //p.y = cy + x*sin(th) + y*cos(th);
  //bb_msg.points.push_back(p);
  
}

visualization_msgs::Marker Cluster::getL1L2VisualisationMessage() {

  visualization_msgs::Marker l1l2_msg;
  //if(!moving){return l1l2_msg;};//cluster not moving-empty msg

  l1l2_msg.header.stamp = ros::Time::now();
  l1l2_msg.header.frame_id  = p_source_frame_name_;
  l1l2_msg.ns = "L1L2";
  l1l2_msg.action = visualization_msgs::Marker::ADD;
  l1l2_msg.pose.orientation.w = 1.0;
  l1l2_msg.type = visualization_msgs::Marker::LINE_STRIP;
  l1l2_msg.id = this->id;
  l1l2_msg.scale.x = 0.1; //line width
  l1l2_msg.color.r = 1.0;
  l1l2_msg.color.g = 0.0;
  l1l2_msg.color.b = 0.0;
  l1l2_msg.color.a = 1.0;

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


visualization_msgs::Marker Cluster::getArrowVisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  //arrow_marker.header.frame_id = p_target_frame_name_;
  arrow_marker.header.stamp = ros::Time::now();
  arrow_marker.ns = "velocities";
  arrow_marker.action = visualization_msgs::Marker::ADD;
  //arrow_marker.scale.x = 0.05;
  //arrow_marker.scale.y = 0.1;  
  arrow_marker.color.a = 1.0;
  arrow_marker.color.g = this->g;
  arrow_marker.color.b = this->b;
  arrow_marker.color.r = this->r;
  arrow_marker.id = this->id;

  arrow_marker.header.frame_id = p_source_frame_name_;
  tf2::Quaternion quat_theta;
  quat_theta.setRPY(0,0,theta);
  quat_theta.normalize();
  //tf2::convert(geo, quat_theta);
  geo = tf2::toMsg(quat_theta);
  arrow_marker.pose.orientation = geo;
  //arrow_marker.pose.orientation.w = 1.0;    
  arrow_marker.pose.position.x = closest_corner_point.first;
  arrow_marker.pose.position.y = closest_corner_point.second;
  arrow_marker.scale.x = 0.2;
  arrow_marker.scale.y = 0.1;  
 
  //geometry_msgs::Point p;
  //p.x = map_kf.state()[0]; 
  //p.y = map_kf.state()[1]; 
  //p.z = 0;
  //arrow_marker.points.push_back(p);

  //p.x = map_kf.state()[0] + map_kf.state()[2]; 
  //p.y = map_kf.state()[1] + map_kf.state()[3]; 
  //p.z = 0;
  //arrow_marker.points.push_back(p);
  return arrow_marker;
}

nav_msgs::Path Cluster::getTrajectory(){
  nav_msgs::Path empty_traj;
  if(!moving){return empty_traj;};
  return trajectory_;
}  


 visualization_msgs::Marker Cluster::getClosestCornerPointVisualisationMessage() {

  visualization_msgs::Marker corner_msg;
  corner_msg.type = visualization_msgs::Marker::POINTS;
  corner_msg.header.frame_id = p_source_frame_name_;
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


 visualization_msgs::Marker Cluster::getCenterVisualisationMessage() {

  visualization_msgs::Marker fcorner_marker;
  fcorner_marker.type = visualization_msgs::Marker::POINTS;
  fcorner_marker.header.frame_id = p_source_frame_name_;
  fcorner_marker.header.stamp = ros::Time::now();
  fcorner_marker.ns = "points";
  fcorner_marker.action = visualization_msgs::Marker::ADD;
  fcorner_marker.pose.orientation.w = 1.0;    
  fcorner_marker.scale.x = 0.07;
  fcorner_marker.scale.y = 0.07;  
  fcorner_marker.color.a = 1.0;
  fcorner_marker.color.g = 1;
  fcorner_marker.color.b = 0;
  fcorner_marker.color.r = 0;
  fcorner_marker.id = this->id;


  geometry_msgs::Point p;
  ////Clockwise Point Switching
  //double x,y;
  //x = closest_corner_point.first;
  //y = closest_corner_point.second;
  p.x = tracker.dynamic.state()(0); 
  p.y = tracker.dynamic.state()(1); 
  fcorner_marker.points.push_back(p);
  th1 = theta;
  th2 = tracker.shape.state()(2);
  for (unsigned int i = 0; i < 5; ++i) {
    //tracker.ClockwisePointSwitch();
    p.x = tracker.dynamic.state()(0); 
    p.y = tracker.dynamic.state()(1); 
    fcorner_marker.points.push_back(p);
  }

  //CounterClockwise Point Switching
  double x,y;
  //geometry_msgs::Point p;
  x = closest_corner_point.first;
  y = closest_corner_point.second;
  p.x = x; 
  p.y = y; 
  const double pi = 3.141592653589793238463; 
  fcorner_marker.points.push_back(p);
  for (unsigned int i = 0; i < 3; ++i) {
    x = x + L2 * sin(theta);
    y = y - L2 * cos(theta);
    double temp = L1;
    L1 = L2;
    L2 = temp;
    theta = theta + pi / 2;
    p.x = x; 
    p.y = y; 
    fcorner_marker.points.push_back(p);
  }
  return fcorner_marker;
}

visualization_msgs::Marker Cluster::getClusterVisualisationMessage() {
  visualization_msgs::Marker cluster_vmsg;
  if(!moving){return cluster_vmsg;};//cluster not moving-empty msg
  cluster_vmsg.header.frame_id  = "laser";
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

  if(!moving){return line_msg;};//cluster not moving-empty msg

  line_msg.header.stamp = ros::Time::now();
  line_msg.header.frame_id  = "/laser";
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
  if(pointListOut.size()>3){moving=false;};
  if(pointListOut.size()==1){
  for(unsigned int k=0; k<pointListOut.size()-1;++k){
    l = sqrt(pow(pointListOut[k+1].first - pointListOut[k].first,2) + pow(pointListOut[k+1].second - pointListOut[k].second,2));
    if(l>0.8){moving = false;};
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
