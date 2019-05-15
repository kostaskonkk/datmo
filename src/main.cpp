#include "ros/ros.h"
#include "datmo.h"
int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "datmo");

  //Create an object of class datmo; 
  datmo  datmo_object;

  ros::spin();

  return 0;
}
