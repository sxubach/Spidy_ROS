
#include "ros/ros.h"
#include <stdlib.h>
#include <ctime>
#include "std_msgs/String.h"
#include "Algorithm_lib/NEAT.h"
#include <sstream>
#include "Algorithm_lib/debug.h"

#include <actionlib/client/simple_action_client.h>

#include <communication_pkg/sensors.h>
#include <communication_pkg/PWMAction.h>

typedef actionlib::SimpleActionClient<communication_pkg::PWMAction> PWMAction_def;

PWMAction_def Actuator("action/pwm", true);

void sensorCallback(communication_pkg::sensors msg){
  //Sensor


}

void sendPWM(char *pwm)
{
	communication_pkg::PWMGoal goal;

    for (int i=0;i<12;i++){
      goal.pwm[i] = pwm[i];
      //printf("goal.pwm[%d] = %d\n",i,pwm_desired[i]);
    }
  Actuator.sendGoal(goal);
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

  ros::Subscriber sensor = n.subscribe("Arduino/sensors",1000, sensorCallback);

  #ifdef DEBUG_H_INCLUDED

  ROS_INFO("Init pub");

  #endif //DEBUG_H_INCLUDED


  //Initialize actionlib


  Actuator.waitForServer();

  char pwm_desired[12]={60,90,60,65, 45,30,45,20, 60,0,85,60};



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
