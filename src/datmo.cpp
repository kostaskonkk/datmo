#include "datmo.h"


void datmo::pubPosesArrayVehicles(){

  geometry_msgs::PoseArray poseArray;


  for(unsigned int i = 0; i <clusters.size(); ++i){
    poseArray.poses.push_back(clusters[i].getPose());

  }

  poseArray.header.stamp = ros::Time::now();
  poseArray.header.frame_id = "/laser";

  vehicles_pub.publish(poseArray);

}


void datmo::callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
//Clustering


  if (time > ros::Time::now().sec)
  {
    clusters.clear();
  }
  // time = ros::Time::now().sec;

  scan = *scan_in;

  // delete all the markers
  visualization_msgs::Marker marker;
  marker.action =3;
  marker_pub.publish(marker);

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


      //Find sum of x and y
      sum_x = sum_x + groups[i][l].first;
      sum_y = sum_y + groups[i][l].second;
      // }

    }

    mean_x = sum_x / groups[i].size();
    mean_y = sum_y / groups[i].size();


    for(unsigned int j=0;j<clusters.size();++j){
      if( abs( mean_x - clusters[j].mean_x() ) < 0.25 && abs( mean_y - clusters[j].mean_y() ) < 0.25){
        //update Cluster
        g_matched[i] = true, c_matched[j] = true;
        clusters[j].update(groups[i]);

        visualization_msgs::Marker viz_line;
        visualization_msgs::Marker viz_point;
        // visualization_msgs::Marker viz_arrow;
        visualization_msgs::Marker viz_cluster;
        visualization_msgs::Marker viz_saved_cluster;

        // updated_clusters = clusters[j].getVisualisationMessageSavedClusters();
        // updated_clusters.ns = "updated_clusters";
        // marker_pub.publish(updated_clusters);

        viz_line = clusters[j].getLineVisualisationMessage();
        viz_point= clusters[j].getPointVisualisationMessage();
        // viz_arrow= clusters[j].getArrowVisualisationMessage();
        viz_cluster= clusters[j].getClusterVisualisationMessage();
        viz_saved_cluster= clusters[j].getSavedClusterVisualisationMessage();


        pubPosesArrayVehicles();

        marker_pub.publish(viz_line);
        marker_pub.publish(viz_point);
        // marker_pub.publish(viz_arrow);
        marker_pub.publish(viz_cluster);
        marker_pub.publish(viz_saved_cluster); 
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
      Cluster cl(cclusters, groups[i]);
      cl.update(groups[i]);
      cclusters++;
      clusters.push_back(cl);
      // visualization_msgs::Marker new_clusters;
      // new_clusters = cl.getVisualisationMessageSavedClusters();
      // new_clusters.ns = "new_clusters";

      // marker_pub.publish(new_clusters);
    } 
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









