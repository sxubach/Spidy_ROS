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

#include "Algorithm_lib/debug_algorithm.h"

#define avg_window 20

char pwm_rest[12]={60,90,120,65, 45,30,45,20, 60,0,85,60};
char pwm_desired[12]={60,90,120,65, 45,30,45,20, 60,0,85,60};
char pwm_current[12]={60,90,120,65, 45,30,45,20, 60,0,85,60};
float distance_U = 0;
int accel_X = 0, sensor_accel_X = 0, g_accel_X = 0, old_accel_X[avg_window];
int accel_Y = 0, sensor_accel_Y = 0, g_accel_Y = 0, old_accel_Y[avg_window];
int accel_Z = 0, sensor_accel_Z = 0, g_accel_Z = 0, old_accel_Z[avg_window];
int gyro_X = 0, sensor_gyro_X=0, g_gyro_X = 0, old_gyro_X[avg_window];
int gyro_Y = 0, sensor_gyro_Y=0, g_gyro_Y = 0, old_gyro_Y[avg_window];
int gyro_Z = 0, sensor_gyro_Z=0, g_gyro_Z = 0, old_gyro_Z[avg_window];

//int caca;

float vel_X=0, vel_Y=0, vel_Z=0;
float X=0, Y=0, Z=0;
float rot_X=0, rot_Y=0, rot_Z=0;
ros::Time t_stamp;

bool first = true;
int fill_avg = 0;
ros::Time begin, end;
float T = 0.1;

typedef actionlib::SimpleActionClient<communication_pkg::PWMAction> PWMAction_def;
float ts = 0.1;
int time_loop_limit = 100; //(1000*ts)=10 sec

int time_loop = 0;

float currentTime = 0;
float InputVec[16] = {};//Input vector to the network
float OutputVec[12] = {};//Output vector of the network
float StepSize2 = 20; //Size of the possition change each second
float Out_hlim = 180; //Up limit possition
float Out_llim = 0; //Down limit positon

float Fitness = 0;

void set_zeros(){
	distance_U = 0;
	accel_X = 0; sensor_accel_X = 0; g_accel_X = 0;
	accel_Y = 0; sensor_accel_Y = 0; g_accel_Y = 0;
	accel_Z = 0; sensor_accel_Z = 0; g_accel_Z = 0;
	gyro_X = 0, sensor_gyro_X=0, g_gyro_X = 0;
	gyro_Y = 0, sensor_gyro_Y=0, g_gyro_Y = 0;
	gyro_Z = 0, sensor_gyro_Z=0, g_gyro_Z = 0;

	vel_X=0; vel_Y=0; vel_Z=0;
	X=0; Y=0; Z=0;
	rot_X=0, rot_Y=0, rot_Z=0;
	ROS_INFO("Set initial values");
}

void avg_filter(){
	accel_X = sensor_accel_X;
	accel_Y = sensor_accel_Y;
	accel_Z = sensor_accel_Z;

	gyro_X = sensor_gyro_X;
	gyro_Y = sensor_gyro_Y;
	gyro_Z = sensor_gyro_Z;

	for (int i=avg_window-1;i>0;i--){
		old_accel_X[i] = old_accel_X[i-1];
		old_accel_Y[i] = old_accel_Y[i-1];
		old_accel_Z[i] = old_accel_Z[i-1];
		accel_X += old_accel_X[i];
		accel_Y += old_accel_Y[i];
		accel_Z += old_accel_Z[i];

		old_gyro_X[i] = old_gyro_X[i-1];
		old_gyro_Y[i] = old_gyro_Y[i-1];
		old_gyro_Z[i] = old_gyro_Z[i-1];
		gyro_X += old_gyro_X[i];
		gyro_Y += old_gyro_Y[i];
		gyro_Z += old_gyro_Z[i];
	}
	old_accel_X[0] = sensor_accel_X;
	old_accel_Y[0] = sensor_accel_Y;
	old_accel_Z[0] = sensor_accel_Z;

	old_gyro_X[0] = sensor_gyro_X;
	old_gyro_Y[0] = sensor_gyro_Y;
	old_gyro_Z[0] = sensor_gyro_Z;
	
	accel_X = accel_X/avg_window;
	accel_Y = accel_Y/avg_window;
	accel_Z = accel_Z/avg_window;
	
	gyro_X = gyro_X/avg_window;
	gyro_Y = gyro_Y/avg_window;
	gyro_Z = gyro_Z/avg_window;
}

void double_integrate(){
	// Saving gravity force
	if (first){
		first = false;
		g_accel_X = accel_X;
		g_accel_Y = accel_Y;
		g_accel_Z = accel_Z;

		g_gyro_X = gyro_X;
		g_gyro_Y = gyro_Y;
		g_gyro_Z = gyro_Z;
	}
	
	// Preprocesing
	end = ros::Time::now();
	T = end.toSec() - begin.toSec();

	accel_X = accel_X - g_accel_X;
	vel_X = vel_X + T*accel_X;
	X = X + T*vel_X;

	accel_Y = accel_Y - g_accel_Y;
	vel_Y = vel_Y + T*accel_Y;
	Y = Y + T*vel_Y;

	accel_Z = accel_Z - g_accel_Z;
	vel_Z = vel_Z + T*accel_Z;
	Z = Z + T*vel_Z;

	gyro_X -= g_gyro_X;
	rot_X += T*gyro_X;

	gyro_Y -= g_gyro_Y;
	rot_Y += T*gyro_Y;

	gyro_Z -= g_gyro_Z;
	rot_Z += T*gyro_Z;
	
	begin = ros::Time::now();
}

void sensorCallback(communication_pkg::sensors msg){
  //Sensor
	distance_U = msg.distance_U;
	sensor_accel_X = msg.accel_X;
	sensor_accel_Y = msg.accel_Y;
	sensor_accel_Z = msg.accel_Z;
	sensor_gyro_X = msg.gyro_X;
	sensor_gyro_Y = msg.gyro_Y;
	sensor_gyro_Z = msg.gyro_Z;
	t_stamp = msg.t_stamp;
	for (int i=0;i<=11;i++){
		pwm_current[i] = msg.pwm[i];
	}
	if (fill_avg<avg_window);
		fill_avg++;
	avg_filter();
	double_integrate();
	
	/*ROS_INFO("gyro_X = %d, g_gyro_X = %d, rot_X = %f",gyro_X,g_gyro_X,rot_X);
	ROS_INFO("gyro_Y = %d, g_gyro_Y = %d, rot_Y = %f",gyro_Y,g_gyro_Y,rot_Y);
	ROS_INFO("gyro_Z = %d, g_gyro_Z = %d, rot_Z = %f",gyro_Z,g_gyro_Z,rot_Z);
	ROS_INFO(" ");*/
}

int main(int argc, char **argv)
{
	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Debugging active");
	#endif //DEBUG_H_INCLUDED

	ros::init(argc, argv, "Algorithm_node");
	ros::NodeHandle n;
	begin = ros::Time::now();
	ros::Rate loop_rate(1/ts);

	srand (time(NULL));


	//Initialize pub/subs

	ros::Subscriber sensor = n.subscribe("/arduino/sensors",1000, sensorCallback);

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Init pub");
	#endif //DEBUG_H_INCLUDED

	//Initialize actionlib

	PWMAction_def Actuator("/arduino/pwm", true);
	//Actuator.waitForServer();

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Init action");
	#endif //DEBUG_H_INCLUDED

	//Initalization of the pool
	Pool Spidy_pool(13,12);

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Pool created");
	#endif //DEBUG_H_INCLUDED

	//Spidy_pool.initializePool();

	Spidy_pool = customReadFile();
	Spidy_pool.currentSpecies = 0;
	Spidy_pool.currentGenome = 0;

	#ifdef DEBUG_H_INCLUDED
	ROS_INFO("Pool initialized");
	#endif //DEBUG_H_INCLUDED


	begin = ros::Time::now();
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

		memcpy(pwm_desired,pwm_rest, 12);
		g_accel_X = 0; g_accel_Y = 0; g_accel_Z = 0;
		g_gyro_X = 0; g_gyro_Y = 0; g_gyro_Z = 0;
		fill_avg=0;
		
		while(fill_avg<avg_window){
			ros::spinOnce();
			loop_rate.sleep();
		}
		set_zeros();
		first = true;
	
		while(time_loop<time_loop_limit){
			for(int i=0;i<12;i++)
				//InputVec[i]=pwm_desired[i];
				InputVec[i]=pwm_current[i];
			
			InputVec[12] = vel_X;
			InputVec[13] = vel_Y;
			InputVec[14] = vel_Z;

			currentTime = time_loop*ts;
			InputVec[15] = currentTime;

			Spidy_pool.evaluateCurrent(InputVec,OutputVec);

			//Process outputs
			for(int i=0;i<12;i++)
			{
				OutputVec[i] = OutputVec[i]*StepSize2;
    				//printf("%f,",OutputVec[i]);

				pwm_desired[i] += OutputVec[i]*ts;

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

		//Calculate fitness
		Fitness = sqrt(X*X+Y*Y)/(rot_Z*rot_Z);
		if (X<0 || Y<0)
			Fitness = 0;
		printf("X = %f, Y = %f, Z = %f, Fitness = %f\n",X,Y,Z,Fitness);

		//Evaluation ended

		Spidy_pool.assignfitness(Fitness);

		//Next genome
		if(Spidy_pool.SpeciesVec[Spidy_pool.currentSpecies].GenomesVec.size()==Spidy_pool.currentGenome+1)
		{
    		if(Spidy_pool.SpeciesVec.size()==Spidy_pool.currentSpecies+1){
					#ifdef DEBUG_ALGO_H_INCLUDED
					ROS_INFO("Generating new generation...");
					#endif //DEBUG_ALGO_H_INCLUDED

	        std::stringstream ss2;
	        ss2 << Spidy_pool.generation;
	        std::string generationstr = ss2.str();
	        std::string textfile = "Generations/TestGen" + generationstr +".txt";

	        customWriteFile(Spidy_pool,textfile);

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
