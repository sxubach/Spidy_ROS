
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
float ts = 0.01;
int time_loop_limit = 1000; //(1000*ts)=10 sec

void sensorCallback(communication_pkg::sensors msg){
  //Sensor


}

void sendPWM(char *pwm,PWMAction_def* Actuator)
{
	communication_pkg::PWMGoal goal;

    for (int i=0;i<12;i++){
      goal.pwm[i] = pwm[i];
      //printf("goal.pwm[%d] = %d\n",i,pwm_desired[i]);
    }
  Actuator->sendGoal(goal);
}

int main(int argc, char **argv)
{

  #ifdef DEBUG_H_INCLUDED
  ROS_INFO("Debugging active");
  #endif //DEBUG_H_INCLUDED

  ros::init(argc, argv, "Algorithm_node");

  srand (time(NULL));

  ros::NodeHandle n;

  ros::Rate loop_rate(1/ts);

  int time_loop = 0;

  float currentTime = 0;
  float InputVec[12] = {};//Input vector to the network
  float OutputVec[12] = {};//Output vector of the network
  float StepSize = 5; //Size of the possition change in a ts
  float Out_hlim = 180; //Up limit possition
  float Out_llim = -180; //Down limit positon

  float Fitness = 0;

  float currentPos[12] = {};//Current

  char pwm_desired[12]={60,90,60,65, 45,30,45,20, 60,0,85,60};

  //Initialize pub/subs

  ros::Subscriber sensor = n.subscribe("Arduino/sensors",1000, sensorCallback);

  #ifdef DEBUG_H_INCLUDED
  ROS_INFO("Init pub");
  #endif //DEBUG_H_INCLUDED

  //Initialize actionlib

  PWMAction_def Actuator("action/pwm", true);
  //Actuator.waitForServer();

  #ifdef DEBUG_H_INCLUDED
  ROS_INFO("Init action");
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

    //Start training

    #ifdef DEBUG_ALGO_H_INCLUDED
    ROS_INFO("Specie: %d. Genome: %d",Spidy_pool.currentSpecies,Spidy_pool.currentGenome);
    #endif //DEBUG_ALGO_H_INCLUDED


    //Generate the network for the current Genome
    Spidy_pool.SpeciesVec[Spidy_pool.currentSpecies].GenomesVec[Spidy_pool.currentGenome].generateNetwork();

    #ifdef DEBUG_H_INCLUDED
    ROS_INFO("Network generated");
    #endif //DEBUG_H_INCLUDED

    //Simulation/Execution loop

    time_loop = 0;

    memset(currentPos, 0, sizeof(currentPos));

    while(time_loop<time_loop_limit){

      //Sensor read implementation

      for(int i=0;i<12;i++)
      {
        InputVec[i]=currentPos[i];
      }

      currentTime = time_loop*ts;

      Spidy_pool.evaluateCurrent(InputVec,OutputVec);


      //Process outputs


      for(int i=0;i<12;i++)
      {
        OutputVec[i] = OutputVec[i]*StepSize;

        currentPos[i] += OutputVec[i]*ts;

        if(currentPos[i]>Out_hlim)
        currentPos[i]=Out_hlim;

        if(currentPos[i]<Out_llim)
        currentPos[i]=Out_llim;
      }

      //Send pwm(currentPos)
      //Que cony envio???



      ros::spinOnce();

      loop_rate.sleep();
      time_loop++;
    }

    //Calculate fitness

    //Evaluation ended

      Spidy_pool.assignfitness(Fitness);

    //Next genome
      if(Spidy_pool.SpeciesVec[Spidy_pool.currentSpecies].GenomesVec.size()==Spidy_pool.currentGenome+1)
      {
        if(Spidy_pool.SpeciesVec.size()==Spidy_pool.currentSpecies+1){
            #ifdef DEBUG_ALGO_H_INCLUDED
            ROS_INFO("Generating new generation...");
            #endif //DEBUG_ALGO_H_INCLUDED

            Spidy_pool.newGeneration();

            #ifdef DEBUG_ALGO_H_INCLUDED
            ROS_INFO("New Generation: %d",Spidy_pool.generation);
            #endif //DEBUG_ALGO_H_INCLUDED
        }else{
          Spidy_pool.currentSpecies ++;
          Spidy_pool.currentGenome = 0;
        }
        }else{
          Spidy_pool.currentGenome++;
      }

  }


  return 0;
}
