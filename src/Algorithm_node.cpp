
#include "ros/ros.h"
#include <stdlib.h>
#include <ctime>
#include "std_msgs/String.h"
#include "Algorithm_lib/NEAT.h"
#include <sstream>
#include "Algorithm_lib/debug.h"

void sensorCallback(){
  //Sensor


}

int main(int argc, char **argv)
{

  #ifdef DEBUG_H_INCLUDED

  ROS_INFO("Debugging active");

  #endif //DEBUG_H_INCLUDED

  ros::init(argc, argv, "Algorithm_node");

  srand (time(NULL));

  ros::NodeHandle n;

  ros::Rate loop_rate(1000);

  //Initialize pub/subs

  #ifdef DEBUG_H_INCLUDED

  ROS_INFO("Init pub");

  #endif //DEBUG_H_INCLUDED


  //Initalization of the pool
  Pool Spidy_pool(12,12);

  #ifdef DEBUG_H_INCLUDED

  ROS_INFO("Pool created");

  #endif //DEBUG_H_INCLUDED

  Spidy_pool.initializePool();

  #ifdef DEBUG_H_INCLUDED

  ROS_INFO("Pool initialized");

  #endif //DEBUG_H_INCLUDED


  while (ros::ok())
  {



    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
