#include "datmo.hpp"

Datmo::Datmo(){
  ros::NodeHandle n; 
  ros::NodeHandle n_private("~");
  ROS_INFO("Starting Detection And Tracking of Moving Objects");

  n_private.param("lidar_frame", lidar_frame, string("laser"));
  n_private.param("world_frame", world_frame, string("map"));
  n_private.param("threshold_distance", dth, 0.2);
  n_private.param("euclidean_distance", euclidean_distance, 0.25);
  n_private.param("pub_markers", p_marker_pub, false);
  n_private.param("write_execution_times", w_exec_times, false);


  pub_tracks_mean    = n.advertise<datmo::TrackArray>("tracks/mean", 10);
  pub_tracks_mean_kf = n.advertise<datmo::TrackArray>("tracks/mean_kf", 10);
  pub_tracks_box     = n.advertise<datmo::TrackArray>("tracks/box", 10);
  pub_marker_array   = n.advertise<visualization_msgs::MarkerArray>("marker_array", 10);
  sub_scan = n.subscribe("/scan", 1, &Datmo::callback, this);

  if (w_exec_times) {
    whole.open ("/home/kostas/results/exec_time/whole.csv");
    whole << ("nano,milli\n");
    clustering.open ("/home/kostas/results/exec_time/clustering.csv");
    clustering << ("nano\n");
    rect_fitting.open("/home/kostas/results/exec_time/rect_fitting.csv");
    rect_fitting << ("dur_nano,num_points\n");
    testing.open("/home/kostas/results/exec_time/testing.csv");
    testing << ("clusters\n");
  }

}
Datmo::~Datmo(){
  if (w_exec_times){
  whole.close();
  clustering.close();
  rect_fitting.close();
  testing.close();
  }
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
    
    dt = (ros::Time::now() - time).toSec();
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

    if (w_exec_times) {
      auto cl_dur_nano = chrono::duration_cast<chrono::nanoseconds>(chrono::steady_clock::now() - start);
      clustering << cl_dur_nano.count()<<"\n";
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
    //#pragma omp parallel for
    for(unsigned int p=0; p<pairs.size();++p){
        clusters[pairs[p].first].update(point_clusters[pairs[p].second], 0.1, ego_pose);
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
      if(g_matched[i] == false){
        Cluster cl(cclusters, point_clusters[i], dt, world_frame, ego_pose);
        cclusters++;
        clusters.push_back(cl);
      } 
    }
    
    //Visualizations and msg publications
    visualization_msgs::MarkerArray marker_array;
    datmo::TrackArray mean_track_array; 
    datmo::TrackArray filtered_track_array; 
    datmo::TrackArray box_track_array; 
    datmo::TrackArray obs_track_array; 
    for (unsigned int i =0; i<clusters.size();i++){

      mean_track_array.tracks.push_back(clusters[i].msg_track_mean);
      filtered_track_array.tracks.push_back(clusters[i].msg_track_mean_kf);
      box_track_array.tracks.push_back(clusters[i].msg_track_box);
     
      if (p_marker_pub){
        marker_array.markers.push_back(clusters[i].getBoundingBoxCenterVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getArrowVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaL1VisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaL2VisualisationMessage());
        marker_array.markers.push_back(clusters[i].getThetaBoxVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getClusterVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoundingBoxVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getBoxModelVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getClosestCornerPointVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getLShapeVisualisationMessage());
        marker_array.markers.push_back(clusters[i].getPoseCovariance());
      };

      if (w_exec_times){
        
        //rect_fitting << clusters[i].getRectangleFittingExecutionTime().first<<",";
        rect_fitting << clusters[i].getRectangleFittingExecutionTime().first<<"\n";
      }
    }
    //ros::Time before_time;
    //before_time = ros::Time::now();
    //double duration;
    //for (unsigned int i =0; i<clusters.size();i++){
        //marker_array.markers.push_back(clusters[i].rectangleFitting());
    //}
    //duration = (ros::Time::now() - before_time).toSec();
    //ROS_INFO_STREAM("Seconds: "<<duration);

    pub_marker_array.publish(marker_array);
    pub_tracks_mean.publish(mean_track_array);
    pub_tracks_mean_kf.publish(filtered_track_array);
    pub_tracks_box.publish(box_track_array);
    visualiseGroupedPoints(point_clusters);
    
    //TODO Publish in Rviz upper right corner this information
     //ROS_INFO_STREAM("Groups"<<point_clusters.size()<< "Clusters: "<<clusters.size());
    // ROS_INFO_STREAM("Time"<<ros::Time::now()<<"clusters: "<<clusters.size() << "Filters: "<<filters.size());


     if (w_exec_times) {
       //Store the time difference between start and end
       auto diff = chrono::steady_clock::now() - start;
       auto diff_nano = chrono::duration_cast<chrono::nanoseconds>(diff);
       auto diff_milli = chrono::duration_cast<chrono::milliseconds>(diff);
       whole << diff_nano.count()<<","<<diff_milli.count()<<"\n";
     }
  }
  else{ //If the tf is not possible init all states at 0
    ROS_WARN_STREAM("No transform could be found between "<<lidar_frame<<" and "<<world_frame);
  };

    
}
void Datmo::visualiseGroupedPoints(const vector<pointList>& point_clusters){
  //Publishing the clusters with different colors
  visualization_msgs::MarkerArray marker_array;
  //Populate grouped points message
  visualization_msgs::Marker gpoints;
  gpoints.header.frame_id = world_frame;
  gpoints.header.stamp = ros::Time(0);
  gpoints.ns = "clustered_points";
  gpoints.action = visualization_msgs::Marker::ADD;
  gpoints.pose.orientation.w = 1.0;
  gpoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  gpoints.scale.x = 0.13;
  gpoints.scale.y = 0.13;
  for(unsigned int i=0; i<point_clusters.size(); ++i){

    gpoints.id = cg;
    cg++;

    float randomg = rand() / double(RAND_MAX);
    float randomb = rand() / double(RAND_MAX);
    float randomr = rand() / double(RAND_MAX);
    gpoints.color.g = randomg;
    gpoints.color.b = randomb;
    gpoints.color.r = randomr;
    gpoints.color.a = 1.0;
    gpoints.lifetime = ros::Duration(0.08);
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
    if(isinf(scan.ranges[i]) == 0){

      polar[j][0] = scan.ranges[i]; //first column is the range 
      polar[j][1] = scan.angle_min + i*scan.angle_increment; //second angle in rad
      j++;
    }
  }

  //Complete the circle
  polar[c_points][0] = polar[0][0];
  polar[c_points][1] = polar[0][1];

  float d;

  //dth = 0.25 * (tp_dth+1)/64 ;// 1 for simulation, 0.2 worked quite good fo
  //float k = 0.01;

 //Find clusters based on adaptive threshold distance
 //Probably I should create two flags, since two consecutive points can belong to two independent clusters
  vector<bool> clustered1(c_points+1 ,false); //change to true when it is the first of the cluster
  vector<bool> clustered2(c_points+1 ,false); // change to true when it is clustered by another one

  //lambda, sigma_r| l=0.5,s=0.3 gives three groups for simulation
  //float l = 1; // λ is an acceptable angle for determining the points to be of the same cluster
  //l = l * 0.0174532;   // degree to radian conversion;
  //const float s = 0;   // σr is the standard deviation of the noise of the distance measure
 
  for (unsigned int i=0; i < c_points ; ++i){

    // dth = k * min(polar[i][0],polar[i+1][0]) * (sin(polar[i+1][1] - polar[i][1])) 
    // / (sin(l - (polar[i+1][1] - polar[i][1]))) + s; //Dthreshold
    d = sqrt( pow(polar[i][0],2) + pow(polar[i+1][0],2)-2 * polar[i][0]*polar[i+1][0]*cos(polar[i+1][1] - polar[i][1]));

    if(d<dth) {
      clustered1[i] = true; //both points belong to clusters
      clustered2[i+1] = true;}
    // else clustered[i] = false;
  }

  //If the last(first also) point is clustered and the first is not, make the first clustered
  if(clustered2[c_points] && !clustered2[0]){
     clustered2[0] = true;
     clustered1[0] = false;
  }
  
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
      ++nclus.back();
    }
  i++;
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
        x = polar[j][0] * cos(polar[j][1]); //x = r × cos( θ )
        y = polar[j][0] * sin(polar[j][1]); //y = r × sin( θ )
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

