
#include "ros/ros.h"
#include <stdlib.h>
#include <ctime>
#include "std_msgs/String.h"
#include "NEAT.h"
#include <sstream>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "main");

  //srand (time(NULL));

  ros::NodeHandle n;

  ros::Rate loop_rate(1000);


  //Initalization of the pool

  while (ros::ok())
  {



    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
