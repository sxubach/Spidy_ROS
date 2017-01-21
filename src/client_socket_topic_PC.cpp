
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
char desired_pwm_values[12] = {60,90,120,5, 45,30,45,20, 60,0,85,60};
char current_pwm_values[12] = {60,90,120,5, 45,30,45,20, 60,0,85,60};
float distance_U = 0;
int accel_X = 0, g_accel_X = 0;
int accel_Y = 0, g_accel_Y = 0;
int accel_Z = 0, g_accel_Z = 0;
int gyro_X = 0;
int gyro_Y = 0;
int gyro_Z = 0;

int s,n=1;
struct sockaddr_in addr;
bool raspberry = false;


typedef actionlib::SimpleActionServer<communication_pkg::PWMAction> Server;

void execute(const communication_pkg::PWMGoalConstPtr& goal, Server* as)
{
	char aux[100];
	
	for(int i=0;i<12;i++){
		desired_pwm_values[i] = (*goal).pwm[i];
	}
	//printf("/arduino/pwm service callback executed");

	as->setSucceeded();
}


int connect_socket()
{
	if (raspberry){
		s = socket(AF_INET, SOCK_STREAM, 0);
		if (s==-1)
			perror("Creat socket:");
		else
			printf("Socket created\n");

	    	addr.sin_family = AF_INET;
		addr.sin_port = htons(PORT);
		inet_aton(HOST, &addr.sin_addr);

		if (connect(s,(struct sockaddr *) &addr,sizeof(addr))<0){
			perror("Error connecting socket:");
			exit(0);
		}else{
			printf("Socket connected\n");
		}
	}
	return s;
}

int reconnect(int s,int err)
{
	//n = read(s,buff,len);
	if (err == 0) {
		printf("Socket not connected --> Reconnecting ...\n");
		close(s);
		s = connect_socket();
		n = 1;
		return s;
	} else if (err<0) {
		perror("Read Socket Error");
	}
	return s;
}

int main(int argc, char** argv)
{
	std::string str;
	char buff[256];

	ros::init(argc, argv, "socket_client_topic_server");
	printf("Initializing socket_client_topic_server\n");
	ros::NodeHandle h;
	
	// Load ROS params from launcher
	ros::param::get("/raspberry", raspberry);
	printf("It is %s that the raspberry is connected\n",raspberry ? "true" : "false");

	ros::param::get("/HOST", str);
	strcpy(HOST,str.c_str());
	printf("The raspberry ip is %s\n", HOST);

	// Initialize Action
	Server server(h, "/arduino/pwm", boost::bind(&execute, _1, &server), false);
	server.start();
	printf("Action /arduino/pwm launched\n");
	
	// Initialize Publisher
	ros::Publisher sensor_pub = h.advertise<communication_pkg::sensors>("/arduino/sensors", 1);
	communication_pkg::sensors msg;
	printf("Publisher /arduino/sensors launched\n");


	// Socket communication
	s = connect_socket();
	
	// Main loop
	while(1){	
		// Comunicating through socket
		if (raspberry){
			printf("\n\n\n");
			sprintf(buff,"4");
			write(s,buff,strlen(buff));

		// Ultrasound
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			distance_U = atoi(buff);
			distance_U = float(distance_U)/58;
			printf("distance_U = %f\n",distance_U);
			write(s,"ack.",4);

		// Accelerometer
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			accel_X = atoi(buff);
			printf("accel_X = %d\n",accel_X);
			write(s,"ack.",4);

			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			accel_Y = atoi(buff);
			printf("accel_Y = %d\n",accel_Y);
			write(s,"ack.",4);

			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			accel_Z = atoi(buff);
			printf("accel_Z = %d\n",accel_Z);
			write(s,"ack.",4);

		// Gyroscope
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			gyro_X = atoi(buff);
			printf("gyro_X = %d\n",gyro_X);
			write(s,"ack.",4);
			
			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			gyro_Y = atoi(buff);
			printf("gyro_Y = %d\n",gyro_Y);
			write(s,"ack.",4);

			memset(buff,0,strlen(buff));
			n = read(s,buff,50);
			gyro_Z = atoi(buff);
			printf("gyro_Z = %d\n",gyro_Z);
			write(s,"ack.",4);

		// PWM
			printf("current_pwm_values[] = [");
			for (int i=1;i<=12;i++){
				memset(buff,0,strlen(buff));	
				n = read(s,buff,50);
				current_pwm_values[i-1] = atoi(buff);
				printf("%d,",current_pwm_values[i-1]);
			
				memset(buff,0,strlen(buff));	
				sprintf(buff,"%d",desired_pwm_values[i-1]);
				write(s,buff,strlen(buff));
			}
			printf("]\n");
			printf("Sending desired_pwm_values[] = [");
			for (int i=0;i<=11;i++){		
				printf("%d,",desired_pwm_values[i]);
			}
			printf("]\n");
			
			n = read(s,buff,50);
			memset(buff,0,50);
			s = reconnect(s,n);

		}else{
			printf("distance_U = %f\n",distance_U);
			printf("accel_X = %d\n",accel_X);
			printf("accel_Y = %d\n",accel_Y);
			printf("accel_Z = %d\n",accel_Z);
			printf("gyro_X = %d\n",gyro_X);
			printf("gyro_Y = %d\n",gyro_X);
			printf("gyro_Z = %d\n",gyro_X);

			printf("desired_pwm_values[] = [");
			for (int i=0;i<=11;i++){		
				printf("%d,",desired_pwm_values[i]);
			}
			printf("]\n");
		}

	// Updating publisher values
		msg.distance_U = distance_U;
		msg.accel_X = accel_X;
		msg.accel_Y = accel_Y;
		msg.accel_Z = accel_Z;
		msg.gyro_X = gyro_X;
		msg.gyro_Y = gyro_Y;
		msg.gyro_Z = gyro_Z;
		msg.t_stamp = ros::Time::now();
		for (int i=0;i<=11;i++){		
			msg.pwm[i] = current_pwm_values[i];
		}
		
		sensor_pub.publish(msg);

		ros::spinOnce();
		ros::Duration(0.1).sleep();
	}
	return 0;
}
