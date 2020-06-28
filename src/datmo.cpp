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

#include "datmo.hpp"

Datmo::Datmo(){
  ros::NodeHandle n; 
  ros::NodeHandle n_private("~");
  ROS_INFO("Starting Detection And Tracking of Moving Objects");

  n_private.param("lidar_frame", lidar_frame, string("base_link"));
  n_private.param("world_frame", world_frame, string("map"));
  ROS_INFO("The lidar_frame is: %s and the world frame is: %s", lidar_frame.c_str(), world_frame.c_str());
  n_private.param("threshold_distance", dth, 0.2);
  n_private.param("max_cluster_size", max_cluster_size, 360);
  n_private.param("euclidean_distance", euclidean_distance, 0.25);
  n_private.param("pub_markers", p_marker_pub, false);

  pub_tracks_box_kf     = n.advertise<datmo::TrackArray>("datmo/box_kf", 10);
  pub_marker_array   = n.advertise<visualization_msgs::MarkerArray>("datmo/marker_array", 10);
  sub_scan = n.subscribe("/scan", 1, &Datmo::callback, this);

}

Datmo::~Datmo(){
}
void Datmo::callback(const sensor_msgs::LaserScan::ConstPtr& scan_in){

  // delete all Markers 
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markera;
  marker.action =3;
  markera.markers.push_back(marker);
  pub_marker_array.publish(markera);

  // Only if there is a transform between the world and lidar frame continue
  if(tf_listener.canTransform(world_frame, lidar_frame, ros::Time())){

    //Find position of ego vehicle in world frame, so it can be fed through to the cluster objects
    tf::StampedTransform ego_pose;
    tf_listener.lookupTransform(world_frame, lidar_frame, ros::Time(0), ego_pose);
    
    //TODO implement varying calculation of dt
    dt = 0.08;

    if (time > ros::Time::now()){clusters.clear();}
    time = ros::Time::now();
    auto start = chrono::steady_clock::now();

    vector<pointList> point_clusters_not_transformed;
    Datmo::Clustering(scan_in, point_clusters_not_transformed);

    //Transform Clusters to world_frame
    vector<pointList> point_clusters;
    for (unsigned int i = 0; i < point_clusters_not_transformed.size(); ++i) {
      pointList point_cluster;
      transformPointList(point_clusters_not_transformed[i], point_cluster);
      point_clusters.push_back(point_cluster);
    }


    // Cluster Association based on the Euclidean distance
    // I should check first all the distances and then associate based on the closest distance

    vector<bool> g_matched(point_clusters.size(),false);   // The Group has been matched with a Cluster
    vector<bool> c_matched(clusters.size(),false); // The Cluster object has been matched with a group

    double euclidean[point_clusters.size()][clusters.size()]; // Matrix object to save the euclidean distances

    //Finding mean coordinates of group and associating with cluster Objects
    double mean_x = 0, mean_y = 0;

    for(unsigned int g = 0; g<point_clusters.size();++g){
      double sum_x = 0, sum_y = 0;
        
      for(unsigned int l =0; l<point_clusters[g].size(); l++){
        sum_x = sum_x + point_clusters[g][l].first;
        sum_y = sum_y + point_clusters[g][l].second;
      }
      mean_x = sum_x / point_clusters[g].size();
      mean_y = sum_y / point_clusters[g].size();

      for(unsigned int c=0;c<clusters.size();++c){
        euclidean[g][c] = abs( mean_x - clusters[c].meanX()) + abs(mean_y - clusters[c].meanY()); 
      }
    }

    //Find the smallest euclidean distance and associate if smaller than the threshold 
    vector<pair <int,int>> pairs;
    for(unsigned int c=0; c<clusters.size();++c){
      unsigned int position;
      double min_distance = euclidean_distance;
      for(unsigned int g=0; g<point_clusters.size();++g){
    if(euclidean[g][c] < min_distance){
      min_distance = euclidean[g][c];
      position = g;
    }
      }
      if(min_distance < euclidean_distance){
        g_matched[position] = true, c_matched[c] = true;
        pairs.push_back(pair<int,int>(c,position));
      }
    }

    //Update Tracked Clusters
    #pragma omp parallel for
    for(unsigned int p=0; p<pairs.size();++p){
      clusters[pairs[p].first].update(point_clusters[pairs[p].second], dt, ego_pose);
    }
       
    //Delete Not Associated Clusters
    unsigned int o=0;
    unsigned int p = clusters.size();
    while(o<p){
      if(c_matched[o] == false){

        std::swap(clusters[o], clusters.back());
        clusters.pop_back();

        std::swap(c_matched[o], c_matched.back());
        c_matched.pop_back();

        o--;
        p--;
      }
    o++;
    }

    // Initialisation of new Cluster Objects
    for(unsigned int i=0; i<point_clusters.size();++i){
      if(g_matched[i] == false && point_clusters[i].size()< max_cluster_size){
	Cluster cl(cclusters, point_clusters[i], dt, world_frame, ego_pose);
	cclusters++;
	clusters.push_back(cl);
      } 
    }
    
    //Visualizations and msg publications
    visualization_msgs::MarkerArray marker_array;
    datmo::TrackArray track_array_box_kf; 
    for (unsigned int i =0; i<clusters.size();i++){

      track_array_box_kf.tracks.push_back(clusters[i].msg_track_box_kf);
     
      if (p_marker_pub){
        marker_array.markers.push_back(clusters[i].getClosestCornerPointVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoundingBoxCenterVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getArrowVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaL1VisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaL2VisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaBoxVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getClusterVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoundingBoxVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoxModelKFVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getLShapeVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getLineVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoxSolidVisualisationMessage());
      }; 
    }

    pub_marker_array.publish(marker_array);
    pub_tracks_box_kf.publish(track_array_box_kf);
    visualiseGroupedPoints(point_clusters);
    
  }
  else{ //If the tf is not possible init all states at 0
    ROS_WARN_DELAYED_THROTTLE(1 ,"No transform could be found between %s and %s", lidar_frame.c_str(), world_frame.c_str());
  };
}
void Datmo::visualiseGroupedPoints(const vector<pointList>& point_clusters){
  //Publishing the clusters with different colors
  visualization_msgs::MarkerArray marker_array;
  //Populate grouped points message
  visualization_msgs::Marker gpoints;
  gpoints.header.frame_id = world_frame;
  gpoints.header.stamp = ros::Time::now();
  gpoints.ns = "clustered_points";
  gpoints.action = visualization_msgs::Marker::ADD;
  gpoints.pose.orientation.w = 1.0;
  gpoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  gpoints.scale.x = 0.04;
  gpoints.scale.y = 0.04;
  for(unsigned int i=0; i<point_clusters.size(); ++i){

    gpoints.id = cg;
    cg++;
    gpoints.color.g = rand() / double(RAND_MAX);
    gpoints.color.b = rand() / double(RAND_MAX);
    gpoints.color.r = rand() / double(RAND_MAX);
    gpoints.color.a = 1.0;
    //gpoints.lifetime = ros::Duration(0.08);
    for(unsigned int j=0; j<point_clusters[i].size(); ++j){
      geometry_msgs::Point p;
      p.x = point_clusters[i][j].first;
      p.y = point_clusters[i][j].second;
      p.z = 0;
      gpoints.points.push_back(p);
    }
    marker_array.markers.push_back(gpoints);
    gpoints.points.clear();
  }
  pub_marker_array.publish(marker_array);

}
void Datmo::Clustering(const sensor_msgs::LaserScan::ConstPtr& scan_in, vector<pointList> &clusters){
  scan = *scan_in;


  int cpoints = 0;
  
  //Find the number of non inf laser scan values and save them in c_points
  for (unsigned int i = 0; i < scan.ranges.size(); ++i){
    if(isinf(scan.ranges[i]) == 0){
      cpoints++;
    }
  }
  const int c_points = cpoints;

  int j = 0;
  vector< vector<float> > polar(c_points +1 ,vector<float>(2)); //c_points+1 for wrapping
  for(unsigned int i = 0; i<scan.ranges.size(); ++i){
    if(!isinf(scan.ranges[i])){
      polar[j][0] = scan.ranges[i]; //first column is the range 
      polar[j][1] = scan.angle_min + i*scan.angle_increment; //second angle in rad
      j++;
    }
  }

  //Complete the circle
  polar[c_points] = polar[0];

  //Find clusters based on adaptive threshold distance
  float d;

 //There are two flags, since two consecutive points can belong to two independent clusters
  vector<bool> clustered1(c_points+1 ,false); //change to true when it is the first of the cluster
  vector<bool> clustered2(c_points+1 ,false); // change to true when it is clustered by another one

  float l = 45; // λ is an acceptable angle for determining the points to be of the same cluster
  l = l * 0.0174532;   // degree to radian conversion;
  const float s = 0;   // σr is the standard deviation of the noise of the distance measure
  for (unsigned int i=0; i < c_points ; ++i){
    double dtheta = polar[i+1][1]- polar[i][1];
    double adaptive = min(polar[i][0],polar[i+1][0]) * (sin(dth)) / (sin(l - (dth))) + s; //Dthreshold
    d = sqrt( pow(polar[i][0],2) + pow(polar[i+1][0],2)-2 * polar[i][0]*polar[i+1][0]*cos(polar[i+1][1] - polar[i][1]));
    //ROS_INFO_STREAM("distance: "<<dth<<", adapt: "<<adaptive<<", dtheta: "<<dtheta);
    //if(polar[i+1][1]- polar[i][1]<0){
      //ROS_INFO_STREAM("problem");
    //}

    if(d<dth) {
      clustered1[i] = true; //both points belong to clusters
      clustered2[i+1] = true;}
  }

  clustered2[0] = clustered2[c_points];
  
  //Going through the points and finding the beginning of clusters and number of points
  vector<int> begin; //saving the first index of a cluster
  vector<int> nclus; //number of clustered points
  int i =0;
  bool flag = true; // flag for not going back through the stack 

  while(i<c_points && flag==true){

    if (clustered1[i] == true && clustered2[i] == false && flag == true){
      begin.push_back(i);
      nclus.push_back(1);
      while(clustered2[i+1] == true && clustered1[i+1] == true ){
	i++;
	++nclus.back();
	if(i==c_points-1 && flag == true){
	  i = -1;
	  flag = false;
	}
      }
      ++nclus.back();//take care of 0 1 flags - last of the cluster
    }
  i++;
  }
  // take care of last point being beginning of cluster
  if(clustered1[cpoints-1]== true and clustered2[c_points-1] == false){
      begin.push_back(cpoints-1);
      nclus.push_back(1);
      i = 0;
      while(clustered2[i] == true && clustered1[i] == true ){
	i++;
	++nclus.back();
      }

  }

  polar.pop_back(); //remove the wrapping element
  int len = polar.size();

  for(unsigned int i=0; i<begin.size(); ++i){

    pointList cluster;

    double x,y;
    int j =begin[i];
    bool fl = true; // flag for not going back through the stack 

    while (j<nclus[i]+begin[i]){
      if(j== len && fl == true) fl = false;
      if (fl == true)
      {
        x = polar[j][0] * cos(polar[j][1]);       //x = r × cos( θ )
        y = polar[j][0] * sin(polar[j][1]);       //y = r × sin( θ )
      }
      else{
       x = polar[j-len][0] *cos(polar[j-len][1]); //x = r × cos( θ )
       y = polar[j-len][0] *sin(polar[j-len][1]); //y = r × sin( θ ) 
      }
      cluster.push_back(Point(x, y));
      ++j;
    }
    clusters.push_back(cluster);
  }
}
void Datmo::transformPointList(const pointList& in, pointList& out){
  //This funcion transforms pointlist between coordinate frames and it is a wrapper for the
  //transformPoint function
  //There is not try catch block because it is supposed to be already encompassed into one
  
  geometry_msgs::PointStamped point_in, point_out;
  Point point; 
  point_in.header.frame_id = lidar_frame;
  point_in.header.stamp = ros::Time(0);
  for (unsigned int i = 0; i < in.size(); ++i) {
    point_in.point.x = in[i].first;
    point_in.point.y = in[i].second;
    tf_listener.transformPoint(world_frame, point_in , point_out);
    point.first = point_out.point.x;
    point.second= point_out.point.y;
    out.push_back(point);
  }
}
