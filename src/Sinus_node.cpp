#include "ros/ros.h"
#include <stdlib.h>
#include <ctime>
#include "std_msgs/String.h"
#include <sstream>
#include <actionlib/client/simple_action_client.h>
#include <communication_pkg/sensors.h>
#include <communication_pkg/PWMAction.h>
#include "Algorithm_lib/debug_algorithm.h"
#include <math.h>

#define PI 3.141592653589793238462643

//char pwm_rest[12]={60,90,120,65, 45,30,45,20, 60,0,85,60};
char pwm_rest[12]={60,90,120,65, 45,30,45,20, 60,0,85,60};
char pwm_desired[12]={60,90,120,65, 45,30,45,20, 60,0,85,60};
char pwm_current[12]={60,90,120,65, 45,30,45,20, 60,0,85,60};
float distance_U = 0;
int accel_X = 0, g_accel_X = 0;
int accel_Y = 0, g_accel_Y = 0;
int accel_Z = 0, g_accel_Z = 0;
int gyro_X = 0;
int gyro_Y = 0;
int gyro_Z = 0;

float vel_X=0, vel_Y=0, vel_Z=0;
float X=0, Y=0, Z=0;
ros::Time t_stamp;

bool first = true;
ros::Time begin, end;
float T = 0.5;

typedef actionlib::SimpleActionClient<communication_pkg::PWMAction> PWMAction_def;
float ts = 0.1;
char leg_group_1, leg_group_2;


void sensorCallback(communication_pkg::sensors msg){
  //Sensor
	distance_U = msg.distance_U;
	accel_X = msg.accel_X;
	accel_Y = msg.accel_Y;
	accel_Z = msg.accel_Z;
	gyro_X = msg.gyro_X;
	gyro_Y = msg.gyro_Y;
	gyro_Z = msg.gyro_Z;
	t_stamp = msg.t_stamp;
	for (int i=0;i<=11;i++){
		pwm_current[i] = msg.pwm[i];
	}

	// Saving gravity force
	if (first){
		first = false;
		g_accel_X = accel_X;
		g_accel_Y = accel_Y;
		g_accel_Z = accel_Z;
	}
	
	// Preprocesing
	end = ros::Time::now();
	T = end.toSec() - begin.toSec();
	printf("execution time T=%f\n",T);

	accel_X = accel_X - g_accel_X;
	vel_X = vel_X + T*accel_X;
	X = X + T*vel_X;

	accel_Y = accel_Y - g_accel_Y;
	vel_Y = vel_Y + T*accel_Y;
	Y = Y + T*vel_Y;

	accel_Z = accel_Z - g_accel_Z;
	vel_Z = vel_Z + T*accel_Z;
	Z = Z + T*vel_Z;
	
	begin = ros::Time::now();

}


int main(int argc, char **argv)
{

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Debugging active");
	#endif //DEBUG_H_INCLUDED

	ros::init(argc, argv, "Sinus_node");

	srand (time(NULL));

	ros::NodeHandle n;

	ros::Rate loop_rate(1/ts);

	int time_loop = 0;

	float currentTime = 0;
	float InputVec[13] = {};//Input vector to the network
	float OutputVec[12] = {};//Output vector of the network
	float StepSize = 20; //Size of the possition change each second
	float Out_hlim = 180; //Up limit possition
	float Out_llim = -180; //Down limit positon

	float Fitness = 0;



	//Initialize pub/subs

	ros::Subscriber sensor = n.subscribe("/arduino/sensors",1000, sensorCallback);
	ROS_INFO("Init pub");
	PWMAction_def Actuator("/arduino/pwm", true);
	ROS_INFO("Init action");

	time_loop = 0;
	while (ros::ok())
	{
		memcpy(InputVec,pwm_desired, 12);

		currentTime = time_loop*ts;
		InputVec[12] = currentTime;

		//Spidy_pool.evaluateCurrent(InputVec,OutputVec);
		//leg_group_1 = sin((time_loop%180)*PI/180);
		//leg_group_2 = cos((time_loop%180)*PI/180);
		

		//Process outputs
		for(int i=0;i<12;i++)
		{
			pwm_desired[i] += 1;

			if(pwm_desired[i]>Out_hlim)
				pwm_desired[i]=Out_hlim;

			if(pwm_desired[i]<Out_llim)
				pwm_desired[i]=Out_llim;
		}

		communication_pkg::PWMGoal goal;
		for (int i=0;i<12;i++){
			goal.pwm[i] = pwm_desired[i];
			//printf("goal.pwm[%d] = %d\n",i,pwm_desired[i]);
		}
		Actuator.sendGoal(goal);


		ros::spinOnce();

		loop_rate.sleep();
		time_loop++;
	}

  return 0;
}
