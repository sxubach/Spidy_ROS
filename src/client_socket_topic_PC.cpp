
#include <actionlib/server/simple_action_server.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>
#include "ros/ros.h"

#include <communication_pkg/PWMAction.h>
#include <communication_pkg/sensors.h>


// Socket address
// HOST defined in launcher
#define PORT 8888

char HOST[16];
char pwm_values[12] = {60,90,60,5, 45,30,45,20, 60,0,85,60};
float distance_U = 0;
int accel_X = 0, g_accel_X = 0;
int accel_Y = 0, g_accel_Y = 0;
int accel_Z = 0, g_accel_Z = 0;
int gyro_X = 0, g_gyro_X = 0;
int gyro_Y = 0, g_gyro_Y = 0;
int gyro_Z = 0, g_gyro_Z = 0;

float vel_X=0, vel_Y=0, vel_Z=0; 
float X=0, Y=0, Z=0; 
float T = 0.5;

typedef actionlib::SimpleActionServer<communication_pkg::PWMAction> Server;


void execute(const communication_pkg::PWMGoalConstPtr& goal, Server* as)
{
	char aux[100];
	
	for(int i=0;i<12;i++){
		pwm_values[i] = (*goal).pwm[i];
	}
	sprintf(aux,"/arduino/pwm service callback executed");
	write(1,aux,strlen(aux));

	as->setSucceeded();
}

int main(int argc, char** argv)
{
	std::string str;
	int s,n;
	struct sockaddr_in addr;
	char buff[256];
	bool raspberry = false, first = true;

	ros::init(argc, argv, "socket_client_topic_server");
	ROS_INFO("Initializing socket_client_topic_server\n");
	ros::NodeHandle h;
	
	ros::param::get("/raspberry", raspberry);
	ROS_INFO("It is %s that the raspberry is connected\n",raspberry ? "true" : "false");

	ros::param::get("/HOST", str);
	strcpy(HOST,str.c_str());
	ROS_INFO("The raspberry ip is %s\n", HOST);

	if (raspberry){
		s = socket(AF_INET, SOCK_STREAM, 0);
		if (s==-1)
			perror("Creat socket:");
		else
			ROS_INFO("Socket created\n");
	
	    	addr.sin_family = AF_INET;
		addr.sin_port = htons(PORT);
		inet_aton(HOST, &addr.sin_addr);

		if (connect(s,(struct sockaddr *) &addr,sizeof(addr))<0){
			perror("Error connecting socket:");
			exit(0);
		}else{
			ROS_INFO("Socket connected\n");
		}
	}

	// Initialize Action
	Server server(h, "/arduino/pwm", boost::bind(&execute, _1, &server), false);
	server.start();
	ROS_INFO("Action /arduino/pwm launched\n");
	
	// Initialize Publisher
	ros::Publisher sensor_pub = h.advertise<communication_pkg::sensors>("/arduino/sensors", 1);
	communication_pkg::sensors msg;
	ROS_INFO("Publisher /arduino/sensors launched\n");


	// Main loop
	while(1){	
	// Comunicating through socket
		if (raspberry){
		// Ultrasound
			sprintf(buff,"%c",char(42));
			write(s,buff,strlen(buff));

			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			if (n == 0)
				ROS_INFO("nothing read\n");
			else if (n<0)
				perror("Read Ultrasound Error:");
			distance_U = atoi(buff);
			distance_U = float(distance_U)/58;

		// Accelerometer
			write(s,"ack.",4);
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			if (n == 0)
				ROS_INFO("nothing read\n");
			else if (n<0)
				perror("Read Ultrasound Error:");	
			accel_X = atoi(buff);
			ROS_INFO("accel_X = %d\n",accel_X);

			write(s,"ack.",4);
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			if (n == 0)
				ROS_INFO("nothing read\n");
			else if (n<0)
				perror("Read Ultrasound Error:");
			accel_Y = atoi(buff);
			ROS_INFO("accel_Y = %d\n",accel_Y);

			write(s,"ack.",4);
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			if (n == 0)
				ROS_INFO("nothing read\n");
			else if (n<0)
				perror("Read Ultrasound Error:");
			accel_Z = atoi(buff);
			ROS_INFO("accel_Z = %d\n",accel_Z);

		// Gyroscope
			write(s,"ack.",4);
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			if (n == 0)
				ROS_INFO("nothing read\n");
			else if (n<0)
				perror("Read Ultrasound Error:");
			gyro_X = atoi(buff);
			ROS_INFO("gyro_X = %d\n",gyro_X);
			
			write(s,"ack.",4);
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			if (n == 0)
				ROS_INFO("nothing read\n");
			else if (n<0)
				perror("Read Ultrasound Error:");
			gyro_Y = atoi(buff);
			ROS_INFO("gyro_Y = %d\n",gyro_Y);

			write(s,"ack.",4);
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			if (n == 0)
				ROS_INFO("nothing read\n");
			else if (n<0)
				perror("Read Ultrasound Error:");
			gyro_Z = atoi(buff);
			ROS_INFO("gyro_Z = %d\n",gyro_Z);

		// PWM
			for (int i=1;i<=12;i++){		
				n = read(s,buff,50);
				if (n == 0)
					ROS_INFO("nothing read\n");
				else if (n<0)
					perror("Read Ultrasound Error:");
			
				memset(buff,0,strlen(buff));
				ROS_INFO("Sending pwm_values[%d] = %d\n",i-1,pwm_values[i-1]);
				sprintf(buff,"%c",char(pwm_values[i-1]+10));
				write(s,buff,strlen(buff));
			}
		}else{
			ROS_INFO("distance_U = %f\n",distance_U);
			ROS_INFO("accel_X = %d\n",accel_X);
			ROS_INFO("accel_Y = %d\n",accel_Y);
			ROS_INFO("accel_Z = %d\n",accel_Z);
			ROS_INFO("gyro_X = %d\n",gyro_X);
			ROS_INFO("gyro_Y = %d\n",gyro_X);
			ROS_INFO("gyro_Z = %d\n",gyro_X);

			for (int i=0;i<=11;i++){		
				ROS_INFO("Sending pwm_values[%d] = %d\n",i,pwm_values[i]);
			}

		}
		ROS_INFO("\n\n\n");

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
		vel_X = vel_X + T*accel_X;
		X = X + T*vel_X;
		vel_Y = vel_Y + T*accel_Y;
		Y = Y + T*vel_Y;
		vel_Z = vel_Z + T*accel_Z;
		Z = Z + T*vel_Z;



	// Updating publisher values
		msg.distance_U = distance_U;
		msg.accel_X = accel_X - g_accel_X;
		msg.accel_Y = accel_Y - g_accel_Y;
		msg.accel_Z = accel_Z - g_accel_Z;
		msg.gyro_X = gyro_X - g_gyro_X;
		msg.gyro_Y = gyro_Y - g_gyro_Y;
		msg.gyro_Z = gyro_Z - g_gyro_Y;
		msg.t_stamp = ros::Time::now();
		
		sensor_pub.publish(msg);

		ros::spinOnce();
	}
	return 0;
}
