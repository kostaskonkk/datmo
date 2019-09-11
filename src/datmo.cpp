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
  n_private.param("pub_vehicles_InBox", p_vehicles_InBox_pub, false);
  n_private.param("pub_vehicles_pub", p_vehicles_pub, false);
  n_private.param("pub_vel_vehicles_pub", p_vel_vehicles_pub, false);
  n_private.param("pub_odom_pub", p_odom_pub, false);
  n_private.param("pub_odom_filtered_pub", p_odom_filtered_pub, false);
  n_private.param("pub_trajectories", p_trajectories_pub, false);
  n_private.param("write_execution_times", w_exec_times, false);


  tracks_pub = n.advertise<datmo::TrackArray>("tracks", 1);
  filtered_tracks_pub = n.advertise<datmo::TrackArray>("filtered_tracks", 1);
  box_tracks_pub = n.advertise<datmo::TrackArray>("box_tracks", 1);
  marker_array_pub = n.advertise<visualization_msgs::MarkerArray>("marker_array", 10);
  trajectory_pub = n.advertise<nav_msgs::Path>("trajectories", 1000);
  //tf.waitForTransform(lidar_frame,world_frame, ros::Time(0), ros::Duration(3.0));
  sub_scan = n.subscribe("/scan", 1, &Datmo::callback, this);

  if (w_exec_times) {
    whole.open ("/home/kostas/results/exec_time/whole.csv");
    whole << ("nano,milli\n");
    clustering.open ("/home/kostas/results/exec_time/clustering.csv");
    clustering << ("nano\n");
    rect_fitting.open("/home/kostas/results/exec_time/rect_fitting.csv");
    rect_fitting << ("dur_nano,num_points\n");
  }

}
Datmo::~Datmo(){
  if (w_exec_times){
  whole.close();
  clustering.close();
  rect_fitting.close();
  }
}

void Datmo::callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

  dt = (ros::Time::now() - time).toSec();
  if (time > ros::Time::now()){clusters.clear();}
  time = ros::Time::now();

  auto start = chrono::steady_clock::now();

  vector<pointList> groups;
  Datmo::Clustering(scan_in, groups);

  if (w_exec_times) {
    auto cl_dur_nano = chrono::duration_cast<chrono::nanoseconds>(chrono::steady_clock::now() - start);
    clustering << cl_dur_nano.count()<<"\n";
  }

  // delete all Markers 
  visualization_msgs::Marker marker;
  visualization_msgs::MarkerArray markera;
  marker.action =3;
  markera.markers.push_back(marker);
  marker_array_pub.publish(markera);




  // Cluster Association based on the Euclidean distance
  // I should check first all the distances and then associate based on the closest distance

  vector<bool> g_matched(groups.size(),false);   // The Group has been matched with a Cluster
  vector<bool> c_matched(clusters.size(),false); // The Cluster object has been matched with a group


  //Finding mean coordinates of group and associating with cluster Objects
  double mean_x = 0, mean_y = 0;

  for(unsigned int i = 0; i<groups.size();++i){
    double sum_x = 0, sum_y = 0;
      
    for(unsigned int l =0; l<groups[i].size(); l++){
      //Find sum of x and y
      sum_x = sum_x + groups[i][l].first;
      sum_y = sum_y + groups[i][l].second;
    }
    mean_x = sum_x / groups[i].size();
    mean_y = sum_y / groups[i].size();

    for(unsigned int j=0;j<clusters.size();++j){
      if( abs( mean_x - clusters[j].meanX() ) < euclidean_distance && abs( mean_y - clusters[j].meanY() ) < euclidean_distance){
        //update Cluster
        g_matched[i] = true, c_matched[j] = true;
        clusters[j].update(groups[i], 0.1, tf_);
      }
    }
  }


  //// Data Association based on the Mahalanobis distance
  //// I should check first all the distances and then associate based on the closest distance

   //Vector2d zD, vD, xy;
   //vector<bool> l_matched(l_shapes.size(),false); // The L-Shape has been matched with a filter
   //vector<bool> f_matched(filters.size(),false);  // The Filter object has been matched with an L-shape

   ////Find the shortest distance
   ////TODO Augment the shortest distance to include the other states as well
   //for(unsigned int i=0; i<l_shapes.size();++i){
     //zD << l_shapes[i][0], l_shapes[i][1];

     //for(unsigned int j=0;j<filters.size();++j){
       //vD = zD - filters[j].C * filters[j].state();
       //if( abs(vD(0)) < 0.15 && abs(vD(1)) < 0.15){
       ////update Kalman Filter
         //l_matched[i] = true, f_matched[j] = true;
         //filters[j].update(zD);
         //xy = filters[j].C*filters[j].state();
       //}
     //}
   //}
 
  // Delete not associated Clusters
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
  for(unsigned int i=0; i<groups.size();++i){
    if(g_matched[i] == false){
      Cluster cl(cclusters, groups[i], dt, tf_, lidar_frame, world_frame);
      cclusters++;
      clusters.push_back(cl);
    } 
  }
  //Visualizations
  visualization_msgs::MarkerArray marker_array;
  datmo::TrackArray track_array; 
  datmo::TrackArray filtered_track_array; 
  datmo::TrackArray box_track_array; 
  for (unsigned int i =0; i<clusters.size();i++){

    //ROS_INFO_STREAM("avx="<<clusters[i].avx<<"avy="<<clusters[i].avy); 
    track_array.tracks.push_back(clusters[i].track_msg);
    box_track_array.tracks.push_back(clusters[i].box_track_msg);
    filtered_track_array.tracks.push_back(clusters[i].filtered_track_msg);
    //if (p_vehicles_InBox_pub){pubPosesArrayVehiclesInsideBox(1);};
    //if (p_vehicles_pub){pubPosesArrayVehicles();};
    //if (p_vel_vehicles_pub){pubVelArrayVehicles();};
    //if (p_trajectories_pub){pubTrajectories();};
   
    if (p_marker_pub){
      //marker_array.markers.push_back(clusters[i].getLineVisualisationMessage());
      marker_array.markers.push_back(clusters[i].getCenterVisualisationMessage());
      marker_array.markers.push_back(clusters[i].getArrowVisualisationMessage());
      marker_array.markers.push_back(clusters[i].getThetaL1VisualisationMessage());
      marker_array.markers.push_back(clusters[i].getThetaL2VisualisationMessage());
      marker_array.markers.push_back(clusters[i].getClusterVisualisationMessage());
      marker_array.markers.push_back(clusters[i].getBoundingBoxVisualisationMessage());
      marker_array.markers.push_back(clusters[i].getBoxModelVisualisationMessage());
      marker_array.markers.push_back(clusters[i].getClosestCornerPointVisualisationMessage());
      marker_array.markers.push_back(clusters[i].getLShapeVisualisationMessage());
    };

    if (w_exec_times){
      
      rect_fitting << clusters[i].getRectangleFittingExecutionTime().first<<",";
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

  marker_array_pub.publish(marker_array);
  tracks_pub.publish(track_array);
  filtered_tracks_pub.publish(filtered_track_array);
  box_tracks_pub.publish(box_track_array);
  //visualiseGroupedPoints(groups);
  
  //TODO Publish in Rviz upper right corner this information
   //ROS_INFO_STREAM("Groups"<<groups.size()<< "Clusters: "<<clusters.size());
  // ROS_INFO_STREAM("Time"<<ros::Time::now()<<"clusters: "<<clusters.size() << "Filters: "<<filters.size());


   if (w_exec_times) {
     //Store the time difference between start and end
     //
     auto diff = chrono::steady_clock::now() - start;
     auto diff_nano = chrono::duration_cast<chrono::nanoseconds>(diff);
     auto diff_milli = chrono::duration_cast<chrono::milliseconds>(diff);
     //file << "Duration="<<diff_nano.count()<<" ns, "<<diff_milli.count()<<" ms\n";
     whole << diff_nano.count()<<","<<diff_milli.count()<<"\n";
     //ROS_INFO_STREAM("Whole="<<diff_nano.count()<<" ns"); 
   }
    
}

void Datmo::visualiseGroupedPoints(const vector<pointList>& groups){
  //Publishing the clusters with different colors
  visualization_msgs::MarkerArray marker_array;
  //Populate grouped points message
  visualization_msgs::Marker gpoints;
  gpoints.header.frame_id = lidar_frame;
  gpoints.header.stamp = ros::Time::now();
  gpoints.ns = "clustered_points";
  gpoints.action = visualization_msgs::Marker::ADD;
  gpoints.pose.orientation.w = 1.0;
  gpoints.type = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  gpoints.scale.x = 0.13;
  gpoints.scale.y = 0.13;
  for(unsigned int i=0; i<groups.size(); ++i){

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
    for(unsigned int j=0; j<groups[i].size(); ++j){
      geometry_msgs::Point p;
      p.x = groups[i][j].first;
      p.y = groups[i][j].second;
      p.z = 0;
      gpoints.points.push_back(p);
    }
    marker_array.markers.push_back(gpoints);
    gpoints.points.clear();
  }
  marker_array_pub.publish(marker_array);

}

void Datmo::pubTrajectories(){
  for(unsigned int i = 0; i <clusters.size(); ++i){
    trajectory_pub.publish(clusters[i].getTrajectory());
  }
}

void Datmo::Clustering(const sensor_msgs::LaserScan::ConstPtr& scan_in, vector<pointList> &clusters)
{
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


