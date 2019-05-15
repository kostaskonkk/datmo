/**
 * Implementation of Cluster class.
*
* @author: Kostas Konstantinidis
* @date: 14.03.2019
*/

#include "cluster.h"


Cluster::Cluster(unsigned long int id, const pointList& new_points ){

  this->id = id;
  this->r = rand() / double(RAND_MAX);
  this->g = rand() / double(RAND_MAX);
  this->b = rand() / double(RAND_MAX);

}


void Cluster::update(const pointList& new_points) {

  clusters.push_back(new_points);
  Cluster::calc_mean(new_points);
  }

geometry_msgs::Pose Cluster::getPose() {

  geometry_msgs::Pose pose;

  pose.position.x = mean_x();
  pose.position.y = mean_y();
  pose.position.z = 0;


  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;

  return pose;


}


visualization_msgs::Marker Cluster::getArrowVisualisationMessage() {

 visualization_msgs::Marker arrow_marker;

 arrow_marker.type = visualization_msgs::Marker::ARROW;
 arrow_marker.header.frame_id = "/laser";
 arrow_marker.header.stamp = ros::Time::now();
 arrow_marker.ns = "velocity";
 arrow_marker.action = visualization_msgs::Marker::ADD;
 arrow_marker.pose.orientation.w = 1.0;    
 arrow_marker.scale.x = 0.2;
 arrow_marker.scale.y = 0.2;  
 arrow_marker.color.a = 1.0;
 arrow_marker.color.g = this->g;
 arrow_marker.color.b = this->b;
 arrow_marker.color.r = this->r;
 arrow_marker.id = this->id;

 geometry_msgs::Point p;
 p.x = mean_x(); 
 p.y = mean_y(); 
 p.z = 0;
 arrow_marker.points.push_back(p);

 p.x = mean_x()+1; 
 p.y = mean_y()+1; 
 p.z = 0;
 arrow_marker.points.push_back(p);
 return arrow_marker;
}

visualization_msgs::Marker Cluster::getPointVisualisationMessage() {

 visualization_msgs::Marker fcorner_marker;

 fcorner_marker.type = visualization_msgs::Marker::POINTS;
 fcorner_marker.header.frame_id = "/laser";
 fcorner_marker.header.stamp = ros::Time::now();
 fcorner_marker.ns = "point";
 fcorner_marker.action = visualization_msgs::Marker::ADD;
 fcorner_marker.pose.orientation.w = 1.0;    
 fcorner_marker.scale.x = 0.2;
 fcorner_marker.scale.y = 0.2;  
 fcorner_marker.color.a = 1.0;
 fcorner_marker.color.g = this->g;
 fcorner_marker.color.b = this->b;
 fcorner_marker.color.r = this->r;
 fcorner_marker.id = this->id;

 geometry_msgs::Point p;
 p.x = mean_x(); 
 p.y = mean_y(); 
 p.z = 0;
 fcorner_marker.points.push_back(p);

 return fcorner_marker;
}

visualization_msgs::Marker Cluster::getClusterVisualisationMessage() {

  visualization_msgs::Marker cluster_vmsg;
  cluster_vmsg.header.frame_id  = "/laser";
  cluster_vmsg.header.stamp = ros::Time::now();
  cluster_vmsg.ns = "clusters";
  cluster_vmsg.action = visualization_msgs::Marker::ADD;
  cluster_vmsg.pose.orientation.w = 1.0;
  cluster_vmsg.type = visualization_msgs::Marker::POINTS;
  cluster_vmsg.scale.x = 0.07;
  cluster_vmsg.scale.y = 0.07;
  cluster_vmsg.id = this->id;

  cluster_vmsg.color.g = this->g;
  cluster_vmsg.color.b = this->b;
  cluster_vmsg.color.r = this->r;
  cluster_vmsg.color.a = 1.0;


  int last_cluster = clusters.size() -1;
  geometry_msgs::Point p;
 
  for(unsigned int j=0; j<clusters[last_cluster].size(); ++j){
    p.x = clusters[last_cluster][j].first;
    p.y = clusters[last_cluster][j].second;
    p.z = 0;
    cluster_vmsg.points.push_back(p);
  }

 // for(unsigned int i = 0; i<clusters.size();i++)
 //  for(unsigned int j=0; j<clusters[i].size(); ++j){
 //    geometry_msgs::Point p;
 //    p.x = clusters[i][j].first;
 //    p.y = clusters[i][j].second;
 //    p.z = 0;
 //    cluster_vmsg.points.push_back(p);
 //  }
  return cluster_vmsg;
}

visualization_msgs::Marker Cluster::getSavedClusterVisualisationMessage() {

  visualization_msgs::Marker saved_cluster;
  saved_cluster.header.frame_id  = "/laser";
  saved_cluster.header.stamp = ros::Time::now();
  saved_cluster.ns = "saved_cluster";
  saved_cluster.action = visualization_msgs::Marker::ADD;
  saved_cluster.pose.orientation.w = 1.0;
  saved_cluster.type = visualization_msgs::Marker::POINTS;
  saved_cluster.scale.x = 0.07;
  saved_cluster.scale.y = 0.07;
  saved_cluster.id = this->id;

  saved_cluster.color.g = this->g;
  saved_cluster.color.b = this->b;
  saved_cluster.color.r = this->r;
  saved_cluster.color.a = 1.0;


  int last_cluster = clusters.size() -1; 
  for(unsigned int j=0; j<clusters[last_cluster].size(); ++j){
    geometry_msgs::Point p;
    p.x = clusters[last_cluster][j].first;
    p.y = clusters[last_cluster][j].second;
    p.z = 0;
    saved_cluster.points.push_back(p);
  }

 // for(unsigned int i = 0; i<clusters.size();i++)
 //  for(unsigned int j=0; j<clusters[i].size(); ++j){
 //    geometry_msgs::Point p;
 //    p.x = clusters[i][j].first;
 //    p.y = clusters[i][j].second;
 //    p.z = 0;
 //    saved_cluster.points.push_back(p);
 //  }
return saved_cluster;
}

visualization_msgs::Marker Cluster::getLineVisualisationMessage() {

  visualization_msgs::Marker line_msg;

  line_msg.header.stamp = ros::Time::now();
  line_msg.header.frame_id  = "/laser";
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

  pointList last_cluster = clusters.back();


  //Feed the cluster into the Iterative End-Point Fit Function
  //and the l_shape_extractor and then save them into the l_shapes vector
  // Line and L-Shape Extraction


  vector<Point> pointListOut;
  Cluster::RamerDouglasPeucker(last_cluster, 0.1, pointListOut);
  geometry_msgs::Point p;
  for(unsigned int k =0 ;k<pointListOut.size();++k){
    p.x = pointListOut[k].first;
    p.y = pointListOut[k].second;
    p.z = 0;

    line_msg.points.push_back(p);
  }

  return line_msg;

}

void Cluster::calc_mean(const pointList& c){

  double sum_x = 0, sum_y = 0;

  for(unsigned int i = 0; i<c.size(); ++i){

    sum_x = sum_x + c[i].first;
    sum_y = sum_y + c[i].second;
  }

    this->mean_values.first = sum_x / c.size();
    this->mean_values.second= sum_y / c.size();
}
void Cluster::calc_theta(){

    this->theta = atan(this->mean_values.second / this->mean_values.second);
}

double Cluster::PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd){
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
void Cluster::RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out){
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
    double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
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
    RamerDouglasPeucker(firstLine, epsilon, recResults1);
    RamerDouglasPeucker(lastLine, epsilon, recResults2);
 
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
