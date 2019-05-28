#include "datmo.h"

void datmo::callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

dt = t - (ros::Time::now().sec + ros::Time::now().nsec);
t = ros::Time::now().sec + ros::Time::now().nsec;


  if (time > ros::Time::now().sec)
  {
    clusters.clear();
    // filters.clear();
  }
  // time = ros::Time::now().sec;

  scan = *scan_in;

  // delete all the markers
  //visualization_msgs::Marker marker;
  //marker.action =3;
  //marker_pub.publish(marker);

  // delete all the markersArrays
  //visualization_msgs::MarkerArray markera;
  //markera.markers.push_back(marker);
  //marker_array_pub.publish(markera);

  vector<pointList> groups;
  datmo::Clustering(scan_in, groups);



  // Cluster Association based on the Euclidean distance
  // I should check first all the distances and then associate based on the closest distance

  vector<bool> g_matched(groups.size(),false);   // The Group has been matched with a Cluster
  vector<bool> c_matched(clusters.size(),false); // The Cluster object has been matched with a group
  double mean_x = 0, mean_y = 0;


  //Going through all the groups and transforming into Map coordinates, finding Mean coordinates of group 
  //and Associating with cluster Objects

  tf::Pose PoseIn, PoseOut;

  for(unsigned int i = 0; i<groups.size();++i){

    double sum_x = 0, sum_y = 0;

    for(unsigned int l =0; l<groups[i].size(); l++){

      //Transform into map coordinates
      // PoseIn.setOrigin( tf::Vector3(groups[i][l].first, groups[i][l].second,0) );
      // PoseIn.setRotation( tf::Quaternion(0, 0, 0, 1));
      // tf::Stamped<tf::Pose> sPoseIn(PoseIn, ros::Time::now(), "laser");


      // PoseOut.setOrigin( tf::Vector3(0, 0,0) );
      // PoseOut.setRotation( tf::Quaternion(0, 0, 0, 0));
      // tf::Stamped<tf::Pose> sPoseOut(PoseOut, ros::Time::now(), "base_link");


      // if(transformer.canTransform("map","laser",ros::Time::now())){
      //   ROS_INFO_STREAM("can transform");

        // transformer.waitForTransform("laser","base_link",ros::Time::now(),ros::Duration(0.1));
        // transformer.transformPose("base_link",sPoseIn, sPoseOut);
       
      // Working with poses
      // groups[i][l].first = sPoseOut.getOrigin()[0];
      // groups[i][l].second= sPoseOut.getOrigin()[1];

      //Find sum of x and y
      sum_x = sum_x + groups[i][l].first;
      sum_y = sum_y + groups[i][l].second;
      // }

    }

    mean_x = sum_x / groups[i].size();
    mean_y = sum_y / groups[i].size();


    for(unsigned int j=0;j<clusters.size();++j){
      if( abs( mean_x - clusters[j].meanX() ) < 0.25 && abs( mean_y - clusters[j].meanY() ) < 0.25){
        //update Cluster
        g_matched[i] = true, c_matched[j] = true;
        clusters[j].update(groups[i], dt);
        clusters[j].updateTrajectory(tf_);
          

        // updated_clusters = clusters[j].getVisualisationMessageSavedClusters();
        // updated_clusters.ns = "updated_clusters";
        // marker_pub.publish(updated_clusters);
 
        if (p_vehicles_InBox_pub){pubPosesArrayVehiclesInsideBox(1);};
        if (p_vehicles_pub){pubPosesArrayVehicles();};
        if (p_vel_vehicles_pub){pubVelArrayVehicles();};
        if (p_odom_pub){pubFilteredOdomObjects();};
        if (p_odom_filtered_pub){pubOdomObjects();};
        if (p_trajectories_pub){pubTrajectories();};
        
        if (p_marker_pub){

          visualization_msgs::Marker viz_line;
          visualization_msgs::Marker viz_point;
          // visualization_msgs::Marker viz_arrow;
          visualization_msgs::Marker viz_cluster;

          viz_line = clusters[j].getLineVisualisationMessage();
          viz_point= clusters[j].getPointVisualisationMessage();
          // viz_arrow= clusters[j].getArrowVisualisationMessage();
          viz_cluster= clusters[j].getClusterVisualisationMessage();

          marker_pub.publish(viz_line);
          marker_pub.publish(viz_point);
          // marker_pub.publish(viz_arrow);
          marker_pub.publish(viz_cluster);
        };

      }
    }
  }

  // Delete not associated Clusters
  int o=0;
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
      // Construction of a new Cluster Object
      Cluster cl(cclusters, groups[i], dt);
      cclusters++;
      clusters.push_back(cl);
      // visualization_msgs::Marker new_clusters;
      // new_clusters = cl.getVisualisationMessageSavedClusters();
      // new_clusters.ns = "new_clusters";

      // marker_pub.publish(new_clusters);
    } 
  }

for (unsigned int i =0; i<clusters.size();i++){

  // visualization_msgs::Marker updated_clusters;
  // updated_clusters = clusters[i].getVisualisationMessageSavedClusters();
  // updated_clusters.ns = "all_clusters";
  // marker_pub.publish(updated_clusters);

}


// ROS_INFO_STREAM("Groups"<<groups.size()<< "Clusters: "<<clusters.size());

// ROS_INFO_STREAM("Time"<<ros::Time::now()<<"clusters: "<<clusters.size() << "Filters: "<<filters.size());


//Populate line strip and grouped points message
   visualization_msgs::Marker gpoints, line_strip;
   gpoints.header.frame_id = line_strip.header.frame_id = "/laser";

   gpoints.header.stamp = line_strip.header.stamp = ros::Time::now();
   line_strip.ns = "extracted_lines";
   gpoints.ns = "clustered_points";

   gpoints.action = line_strip.action = visualization_msgs::Marker::ADD;
   gpoints.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

   gpoints.type = visualization_msgs::Marker::POINTS;
   line_strip.type = visualization_msgs::Marker::LINE_STRIP;
   // POINTS markers use x and y scale for width/height respectively
   gpoints.scale.x = 0.13;
   gpoints.scale.y = 0.13;
   // Corner points are green
   gpoints.color.g = 1.0f;
   gpoints.color.a = 1.0;
   line_strip.scale.x = 0.1; //line width
   // Line strip is blue
   line_strip.color.b = 1.0;
   line_strip.color.a = 1.0;


  
//Feed the clusters into the Iterative End-Point Fit Function
//and the l_shape_extractor and then save them into the l_shapes vector
  // vector<l_shape> l_shapes;

 visualization_msgs::MarkerArray marker_array;
   for(unsigned int i=0; i<groups.size(); ++i){


    //Publishing the clusters with different colors

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
  //   // Line and L-Shape Extraction


  //   vector<Point> pointListOut;
  //   datmo::RamerDouglasPeucker(groups[i], 0.1, pointListOut);
  //   for(unsigned int k =0 ;k<pointListOut.size();++k){
  //     geometry_msgs::Point p;
  //     p.x = pointListOut[k].first;
  //     p.y = pointListOut[k].second;
  //     p.z = 0;

  //     line_strip.points.push_back(p);
  //   }
  //   if(pointListOut.size() ==3){
  //       vector<double> l_shape;
  //       datmo::l_shape_extractor(pointListOut, l_shape, 0); // Last value is bool visualise
  //       l_shapes.push_back(l_shape);
  //       ++cl;

  //       geometry_msgs::Point p;
  //       p.x = l_shape[0];
  //       p.y = l_shape[1];
  //       p.z = 0;
  //       // corner.points.push_back(p);
  //     }
  //   line_strip.id = cg;
  //   marker_pub.publish(line_strip);
  //   line_strip.points.clear();
  // }

  // for(unsigned int i=0; i<l_shapes.size();++i){
  //   geometry_msgs::Point p;
  //   p.x = l_shapes[i][0];
  //   p.y = l_shapes[i][1];
  //   p.z = 0;
  //   L_pub.publish(p);    
  // }
  

  // Data Association based on the Mahalanobis distance
  // I should check first all the distances and then associate based on the closest distance

  // Vector2d zD, vD, xy;
  // vector<bool> l_matched(l_shapes.size(),false); // The L-Shape has been matched with a filter
  // vector<bool> f_matched(filters.size(),false);  // The Filter object has been matched with an L-shape

  // Find the shortest distance
  // TODO Augment the shortest distance to include the other states as well
  // for(unsigned int i=0; i<l_shapes.size();++i){
  //   zD << l_shapes[i][0], l_shapes[i][1];

  //   for(unsigned int j=0;j<filters.size();++j){
  //     vD = zD - filters[j].C * filters[j].state();
  //     if( abs(vD(0)) < 0.15 && abs(vD(1)) < 0.15){
  //     //update Kalman Filter
  //       l_matched[i] = true, f_matched[j] = true;
  //       filters[j].update(zD);
  //       xy = filters[j].C*filters[j].state();
  //     }
  //   }
  // }

  // ROS_INFO_STREAM("associated: "<<std::accumulate(f_matched.begin(), f_matched.end(), 0)<<"/"<<std::accumulate(l_matched.begin(), l_matched.end(), 0));
  // Delete not associated Kalman Filters
  // unsigned int i=0;
  // unsigned int j = filters.size();
  // while(i<j){
  

  // j--;

  //   if(f_matched[i] == false){
  //     std::swap(filters[i], filters.back());
  //     filters.pop_back();
  //     std::swap(f_matched[i], f_matched.back());
  //     f_matched.pop_back();
  //     i--;
  //     i--;
  //   }
  // i++;
  // }


  // if (f_matched.size()!=filters.size()){
  // ROS_INFO_STREAM("l_shapes"<<l_shapes.size()<<"f_matched"<<f_matched.size()<<"filters.size"<<filters.size());
  // }
  // ROS_INFO_STREAM("associated: "<<std::accumulate(f_matched.begin(), f_matched.end(), 0)<<"/"<<std::accumulate(l_matched.begin(), l_matched.end(), 0));

  // for(unsigned int i=0; i<filters.size();++i){
  //   if(f_matched[i] == false){
  //     filters.erase (filters.begin()+i);
  //   }
  // }
 

      

 //    visualization_msgs::Marker fcorner_marker;
 //    fcorner_marker.type = visualization_msgs::Marker::POINTS;

 //    fcorner_marker.header.frame_id = "/odom";
 //    fcorner_marker.ns = "filter_corner";
 //    fcorner_marker.action = visualization_msgs::Marker::ADD;
 //    fcorner_marker.pose.orientation.w = 1.0;    
 //    fcorner_marker.header.stamp = ros::Time::now();
 //    fcorner_marker.scale.x = 0.4;
 //    fcorner_marker.scale.y = 0.4;  
 //    fcorner_marker.color.a = 1.0;
 // //Visualisation of corner point of Kalman Filters
 // for(unsigned int i=0;i<filters.size();++i){
 //        xy = filters[i].C*filters[i].state();
 //        geometry_msgs::Point p;
 //        p.x = xy(0);
 //        p.y = xy(1);
 //        p.z = 0;

 //        fcorner_marker.id = filters[i].id; 

 //        fcorner_marker.color.r = filters[i].r;
 //        fcorner_marker.color.g = filters[i].g;
 //        fcorner_marker.color.b = filters[i].b;

 //        fcorner_marker.points.push_back(p);
 //        marker_pub.publish(fcorner_marker);
 //        fcorner_marker.points.clear();

 //  }

// ROS_INFO_STREAM("Kf: "<<filters.size()<<"ls: "<<l_shapes.size());



}

// This function expects pointLists of 3 points. The corner point is equal to the middle point. 
// TODO calculate theta from the same axis each time
void datmo::l_shape_extractor(const vector<Point> &pointListIn, vector<double> &l_shape, bool visualise)
{

  l_shape.push_back(pointListIn[1].first); //xcorner
  l_shape.push_back(pointListIn[1].second);//ycorner
  double L1,L2,theta;
  L1 = sqrt(pow(pointListIn[1].first - pointListIn[0].first,2) + pow(pointListIn[1].second - pointListIn[0].second,2));
  L2 = sqrt(pow(pointListIn[2].first - pointListIn[1].first,2) + pow(pointListIn[2].second - pointListIn[1].second,2));
  l_shape.push_back(L1);
  l_shape.push_back(L2);
  theta = atan(pointListIn[1].second - pointListIn[0].second / pointListIn[1].first - pointListIn[0].first);//arctan(dy/dx)
  l_shape.push_back(theta);


  if(visualise == 1){

    visualization_msgs::Marker lshapes_marker;
    lshapes_marker.type = visualization_msgs::Marker::LINE_STRIP;

    lshapes_marker.header.frame_id = "/map";
    lshapes_marker.ns = "L-Shapes";
    lshapes_marker.action = visualization_msgs::Marker::ADD;
    lshapes_marker.id = cl;
    lshapes_marker.pose.orientation.w = 1.0;    
    lshapes_marker.header.stamp = ros::Time::now();
    lshapes_marker.scale.x = 0.1;  //line width
    lshapes_marker.color.g = 0.0f;
    lshapes_marker.color.b = 0.0f;
    lshapes_marker.color.r = 0.8f;
    

    lshapes_marker.color.a = 1.0;

    geometry_msgs::Point p;
    for(unsigned int i=0;i<3;i++){
      p.x = pointListIn[i].first;
      p.y = pointListIn[i].second;
      p.z = 0;
      lshapes_marker.points.push_back(p);
    }

    marker_pub.publish(lshapes_marker);
  }
}

void datmo::Clustering(const sensor_msgs::LaserScan::ConstPtr& scan_in, vector<pointList> &clusters)
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
  for(int i = 0; i<scan.ranges.size(); ++i){
    if(isinf(scan.ranges[i]) == 0){

      polar[j][0] = scan.ranges[i]; //first column is the range 
      polar[j][1] = scan.angle_min + i*scan.angle_increment; //second angle in rad
      j++;
    }
  }

  //Complete the circle
  polar[c_points][0] = polar[0][0];
  polar[c_points][1] = polar[0][1];

  float dth, d;

  dth = 0.25 * (tp_dth+1)/64 ;// 1 for simulation, 0.2 worked quite good fo
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
  //for (unsigned int i=0; i < c_points ; ++i){

    //pointList loner;

    //double x,y;
    //if(clustered1[i] == false && clustered2[i] == false){
      //x = polar[i][0] * cos(polar[i][1]); //x = r × cos( θ )
      //y = polar[i][0] * sin(polar[i][1]); //y = r × sin( θ )
      //loner.push_back(Point(x, y));
    //}
    //clusters.push_back(loner);
  //}

void datmo::pubTrajectories(){
  for(unsigned int i = 0; i <clusters.size(); ++i){
    trajectory_pub.publish(clusters[i].getTrajectory());
  }
}

void datmo::pubOdomObjects(){
  for(unsigned int i = 0; i <clusters.size(); ++i){
    odom_pub.publish(clusters[i].getOdom());
  }
}

void datmo::pubFilteredOdomObjects(){
  for(unsigned int i = 0; i <clusters.size(); ++i){
    odom_filtered_pub.publish(clusters[i].getFilteredOdom());
  }
}

void datmo::pubPosesArrayVehicles(){
  geometry_msgs::PoseArray poseArray;
  for(unsigned int i = 0; i <clusters.size(); ++i){
    poseArray.poses.push_back(clusters[i].getPose());
  }

  poseArray.header.stamp = ros::Time::now();
  poseArray.header.frame_id = "/laser";

  vehicles_pub.publish(poseArray);

}

// void datmo::midi_callback(const std_msgs::Int8::ConstPtr& msg){

//   tp_dth = msg->data;
//   ROS_INFO_STREAM("Clustering_Distance = 0.25 * ("<<tp_dth<<"+1)/64 = "<<0.25 * (tp_dth+1)/64);
// // 0.25 * (tp_dth+1)/64
// }

void datmo::pubPosesArrayVehiclesInsideBox(double halfwidth){

  geometry_msgs::PoseArray poseArrayInBox;


  for(unsigned int i = 0; i <clusters.size(); ++i){

    if(abs(clusters[i].getPose().position.x)< 1 && abs(clusters[i].getPose().position.y)< 1){
      poseArrayInBox.poses.push_back(clusters[i].getPose());
    }

  }

  poseArrayInBox.header.stamp = ros::Time::now();
  poseArrayInBox.header.frame_id = "/laser";

  vehicles_InBox_pub.publish(poseArrayInBox);
}

void datmo::pubVelArrayVehicles(){

  geometry_msgs::PoseArray velArray;


  for(unsigned int i = 0; i <clusters.size(); ++i){
    velArray.poses.push_back(clusters[i].getVel());

  }

  velArray.header.stamp = ros::Time::now();
  velArray.header.frame_id = "/laser";

  vel_vehicles_pub.publish(velArray);

}
