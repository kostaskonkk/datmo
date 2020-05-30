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

Cluster::Cluster(unsigned long int id, const pointList& new_points, const double& dt, const std::string& world_frame, const tf::Transform& ego_pose){

  this->id = id;
  this->r = rand() / double(RAND_MAX);
  this->g = rand() / double(RAND_MAX);
  this->b = rand() / double(RAND_MAX);
  a = 1.0;
  age = 1;
  frame_name = world_frame;

  new_cluster = new_points;

  ego_coordinates.first = ego_pose.getOrigin().getX();
  ego_coordinates.second= ego_pose.getOrigin().getY();


  calcMean(new_points);
  previous_mean_values = mean_values;
  rectangleFitting(new_points);

  LshapeTracker l_shape_tracker_ukf(closest_corner_point.first, closest_corner_point.second, L1, L2, normalize_angle(thetaL1), dt);
  this->Lshape = l_shape_tracker_ukf;
  Lshape.BoxModel(cx, cy, cvx, cvy, th, psi, comega, L1_box, L2_box, length_box, width_box);
  
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

    Lshape.update(thetaL1, closest_corner_point.first, closest_corner_point.second, L1, L2, dt, new_points.size());

  Lshape.BoxModel(cx, cy, cvx, cvy, th, psi, comega, L1_box, L2_box, length_box, width_box);

  populateTrackingMsgs(dt);

}

void Cluster::populateTrackingMsgs(const double& dt){
  // This function populates the datmo/Tracks msgs.

  msg_track_box_kf.id = this->id;
  msg_track_box_kf.odom.header.stamp = ros::Time::now();
  msg_track_box_kf.odom.header.frame_id = frame_name;
  msg_track_box_kf.odom.pose.pose.position.x = cx;
  msg_track_box_kf.odom.pose.pose.position.y = cy;
  msg_track_box_kf.odom.twist.twist.linear.x = cvx;
  msg_track_box_kf.odom.twist.twist.linear.y = cvy;
  msg_track_box_kf.length = length_box;
  msg_track_box_kf.width  = width_box;

  quaternion.setRPY(0, 0, psi);
  msg_track_box_kf.odom.pose.pose.orientation = tf2::toMsg(quaternion);
  msg_track_box_kf.odom.twist.twist.angular.z   = comega;

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
  th = 0.0;
  //TODO make d configurable through Rviz
  unsigned int d = 25;
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

  std::vector<Point> corners;
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
  std::vector<Point> l1l2_list;
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
  
} 
visualization_msgs::Marker Cluster::getBoundingBoxVisualisationMessage() {

  visualization_msgs::Marker bb_msg;
  bb_msg.header.stamp = ros::Time::now();
  bb_msg.header.frame_id  = frame_name;
  bb_msg.ns = "bounding_boxes";
  bb_msg.action = visualization_msgs::Marker::ADD;
  bb_msg.pose.orientation.w = 1.0;
  bb_msg.type = visualization_msgs::Marker::LINE_STRIP;
  bb_msg.id = this->id;
  //bb_msg.scale.x = 0.3; //line width
  bb_msg.scale.x = 0.008; //line width
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
visualization_msgs::Marker Cluster::getBoxModelKFVisualisationMessage() {
  
  visualization_msgs::Marker bb_msg;

  bb_msg.header.stamp = ros::Time::now();
  bb_msg.header.frame_id  = frame_name;
  bb_msg.ns = "box_models_kf";
  bb_msg.action = visualization_msgs::Marker::ADD;
  bb_msg.pose.orientation.w = 1.0;
  bb_msg.type = visualization_msgs::Marker::LINE_STRIP;
  bb_msg.id = this->id;
  bb_msg.scale.x = 0.02; //line width
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
  //l1l2_msg.scale.x = 0.3; //line width
  l1l2_msg.scale.x = 0.1; //line width
  l1l2_msg.color.r = 1;
  l1l2_msg.color.g = 0;
  l1l2_msg.scale.x = 0.1; //line width
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
double Cluster::closenessCriterion(const VectorXd& C1, const VectorXd& C2, const double& d0){
  //Algorithm 4 of "Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners"

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
    min = std::min(D1(i),D2(i));
    d = std::max(min,d0);
    b = b + 1/d;
  }
 
  return b; 
}
visualization_msgs::Marker Cluster::getThetaBoxVisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type            = visualization_msgs::Marker::ARROW;
  arrow_marker.header.stamp    = ros::Time::now();
  arrow_marker.ns              = "thetaBox";
  arrow_marker.action          = visualization_msgs::Marker::ADD;
  arrow_marker.color.a         = 0.5;
  arrow_marker.color.g         = this->g;
  arrow_marker.color.b         = this->b;
  arrow_marker.color.r         = this->r;
  arrow_marker.id              = this->id;
  arrow_marker.pose.position.x = cx;
  arrow_marker.pose.position.y = cy;
  arrow_marker.pose.position.z = 0;
  arrow_marker.scale.x         = length_box;
  arrow_marker.scale.y         = width_box;
  arrow_marker.scale.z         = 0.01;

  arrow_marker.header.frame_id = frame_name;

  //quaternion.setRPY(0,0,orientation);
  quaternion.setRPY(0,0,psi);
  arrow_marker.pose.orientation = tf2::toMsg(quaternion);
 
  return arrow_marker;
}
visualization_msgs::Marker Cluster::getThetaL1VisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.header.frame_id = frame_name;
  arrow_marker.type            = visualization_msgs::Marker::ARROW;
  arrow_marker.header.stamp    = ros::Time::now();
  arrow_marker.ns              = "thetaL1";
  arrow_marker.action          = visualization_msgs::Marker::ADD;
  arrow_marker.color.a         = 1.0;
  arrow_marker.color.r         = 1;
  arrow_marker.color.g         = 0;
  arrow_marker.color.b         = 0;
  arrow_marker.id              = this->id;

  tf2::Quaternion quat_theta;
  quat_theta.setRPY(0,0,thetaL1);
  arrow_marker.pose.orientation = tf2::toMsg(quat_theta);
  arrow_marker.pose.position.x  = closest_corner_point.first;
  arrow_marker.pose.position.y  = closest_corner_point.second;
  arrow_marker.pose.position.z  = 0;
  arrow_marker.scale.x          = L1_box;
  //arrow_marker.scale.x          = 0.1;
  arrow_marker.scale.y          = 0.04;
  arrow_marker.scale.z          = 0.001;
 
  return arrow_marker;
}
visualization_msgs::Marker Cluster::getThetaL2VisualisationMessage() {

  visualization_msgs::Marker arrow_marker;
  arrow_marker.type              = visualization_msgs::Marker::ARROW;
  //arrow_marker.header.frame_id = frame_name;
  arrow_marker.header.stamp      = ros::Time::now();
  arrow_marker.ns                = "thetaL2";
  arrow_marker.action            = visualization_msgs::Marker::ADD;
  arrow_marker.color.a           = 1.0;
  arrow_marker.color.g         = 1;
  arrow_marker.color.g           = 0;
  arrow_marker.color.b           = 0;
  arrow_marker.id                = this->id;

  arrow_marker.header.frame_id = frame_name;
  tf2::Quaternion quat_theta;
  quat_theta.setRPY(0,0,thetaL2);
  //quat_theta.normalize();
  arrow_marker.pose.orientation = tf2::toMsg(quat_theta);
  arrow_marker.pose.position.x  = closest_corner_point.first;
  arrow_marker.pose.position.y  = closest_corner_point.second;
  arrow_marker.scale.x          = L2_box;
  arrow_marker.scale.y          = 0.04;
  arrow_marker.scale.z          = 0.01;
  return arrow_marker;
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
  arrow_marker.scale.x = 0.05;    //Shaft diameter of the arrow
  arrow_marker.scale.y = 0.1;    //Head  diameter of the arrow

  geometry_msgs::Point p;
  p.x = cx; 
  p.y = cy; 
  p.z = 0;
  arrow_marker.points.push_back(p);

  p.x = cx + cvx *1; 
  p.y = cy + cvy *1; 
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
  //corner_msg.scale.x = 0.3;
  //corner_msg.scale.y = 0.3;  
  corner_msg.scale.x = 0.08;
  corner_msg.scale.y = 0.08;  
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
    p.x = cx;
    p.y = cy;
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
  cluster_vmsg.scale.x = 0.02;
  cluster_vmsg.scale.y = 0.02;
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

visualization_msgs::Marker Cluster::getBoxSolidVisualisationMessage() {


  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_name;
  marker.header.stamp = ros::Time::now();
  marker.ns = "boxes";
  marker.id = this->id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = cx;
  marker.pose.position.y = cy;
  marker.pose.position.z = 0;

  quaternion.setRPY(0, 0, psi);
  marker.pose.orientation = tf2::toMsg(quaternion);

  marker.scale.x = length_box;
  marker.scale.y = width_box;
  marker.scale.z = 0.01;

  marker.color.r = this->r;
  marker.color.g = this->g;
  marker.color.b = this->b;
  marker.color.a = 1.0;

  return marker;
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
  line_msg.color.g = this->g;
  line_msg.color.b = this->b;
  line_msg.color.r = this->r;
  line_msg.color.a = 1.0;


  //Feed the cluster into the Iterative End-Point Fit Function
  //and the l_shape_extractor and then save them into the l_shapes vector
  // Line and L-Shape Extraction

  std::vector<Point> pointListOut;
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
void Cluster::ramerDouglasPeucker(const std::vector<Point> &pointList, double epsilon, std::vector<Point> &out){
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
    std::vector<Point> recResults1;
    std::vector<Point> recResults2;
    std::vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
    std::vector<Point> lastLine(pointList.begin()+index, pointList.end());
    ramerDouglasPeucker(firstLine, epsilon, recResults1);
    ramerDouglasPeucker(lastLine, epsilon, recResults2);
 
    // Build the result list
    out.assign(recResults1.begin(), recResults1.end()-1);
    out.insert(out.end(), recResults2.begin(), recResults2.end());
    if(out.size()<2)
      throw std::runtime_error("Problem assembling output");
  } 
  else 
  {
    //Just return start and end points
    out.clear();
    out.push_back(pointList[0]);
    out.push_back(pointList[end]);
  }
}
