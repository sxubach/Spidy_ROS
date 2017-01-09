
#include <communication_pkg/PWMAction.h>
#include <actionlib/server/simple_action_server.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sstream>


// Socket address
//#define HOST2 "192.168.0.55"
#define PORT 8888

char HOST[16];
char pwm_values[12] = {60,90,60,5, 45,30,45,20, 60,0,85,60};// {0,0,0,0,0,0,0,0,0,0,0,0};//
float distance_U = 0;
int accel_X = 0;
int accel_Y = 0;
int accel_Z = 0;
int gyro_X = 0;
int gyro_Y = 0;
int gyro_Z = 0;

typedef actionlib::SimpleActionServer<communication_pkg::PWMAction> Server;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

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
	bool raspberry = false;

	ros::init(argc, argv, "socket_client_topic_server");
	printf("Initializing socket_client_topic_server\n");
	ros::NodeHandle h;
	
	ros::param::get("/raspberry", raspberry);
	printf("It is %s that the raspberry is connected\n",raspberry ? "true" : "false");

	ros::param::get("/HOST", str);
	printf("The raspberry ip is %s\n", str.c_str());
	strcpy(HOST,str.c_str());
	printf("The raspberry ip is %s\n", HOST);

	if (raspberry){
		s = socket(AF_INET, SOCK_STREAM, 0);
		if (s==-1)
			perror("Creat socket:");
		else
			printf("Socket created\n");
	
	    	addr.sin_family = AF_INET;
		addr.sin_port = htons(PORT);
		inet_aton(HOST, &addr.sin_addr);
		//inet_aton(HOST2, &addr.sin_addr);

		if (connect(s,(struct sockaddr *) &addr,sizeof(addr))<0){
			perror("Error connecting socket:");
			exit(0);
		}else{
			printf("Socket connected\n");
		}
	}

	Server server(h, "/arduino/pwm", boost::bind(&execute, _1, &server), false);
	server.start();
	printf("Action /arduino/pwm launched\n");
	

	// Main loop
	while(1){	
		if (raspberry){
		// Ultrasound
			sprintf(buff,"%c",char(100+10));
			write(s,buff,strlen(buff));
			//printf("writen %s in socket\n",buff);	

			memset(buff,0,strlen(buff));
			n = read(s,buff,255);
			distance_U = atoi(buff);
			if (n == 0)		printf("nothing read\n");
			else if (n<0)		perror("Read Error:");	
			distance_U = float(distance_U)/58;
			printf("distance_U = %f\n",distance_U);

		// Accelerometer
			sprintf(buff,"%c",char(101+10));
			write(s,buff,strlen(buff));
			memset(buff,0,strlen(buff));
			n = read(s,buff,255);	
			accel_X = atoi(buff);
			printf("accel_X = %d\n",accel_X);

			sprintf(buff,"%c",char(102+10));
			write(s,buff,strlen(buff));
			memset(buff,0,strlen(buff));
			n = read(s,buff,255);
			accel_Y = atoi(buff);
			printf("accel_Y = %d\n",accel_Y);

			sprintf(buff,"%c",char(103+10));
			write(s,buff,strlen(buff));
			memset(buff,0,strlen(buff));
			n = read(s,buff,255);
			accel_Z = atoi(buff);
			printf("accel_Z = %d\n",accel_Z);

		// Gyroscope
			sprintf(buff,"%c",char(104+10));
			write(s,buff,strlen(buff));
			memset(buff,0,strlen(buff));
			n = read(s,buff,255);
			gyro_X = atoi(buff);
			printf("gyro_X = %d\n",gyro_X);

			sprintf(buff,"%c",char(105+10));
			write(s,buff,strlen(buff));
			memset(buff,0,strlen(buff));
			n = read(s,buff,255);
			gyro_Y = atoi(buff);
			printf("gyro_Y = %d\n",gyro_Y);

			sprintf(buff,"%c",char(106+10));
			write(s,buff,strlen(buff));
			memset(buff,0,strlen(buff));
			n = read(s,buff,255);
			gyro_Z = atoi(buff);
			printf("gyro_Z = %d\n",gyro_Z);

		// PWM
			for (int i=1;i<=12;i++){			
				sprintf(buff,"%c",char(i+10));
				write(s,buff,strlen(buff));
			
				memset(buff,0,strlen(buff));
				read(s,buff,255);
			
				memset(buff,0,strlen(buff));
				printf("Sending pwm_values[%d] = %d\n",i-1,pwm_values[i-1]);
				sprintf(buff,"%c",char(pwm_values[i-1]+10));
				write(s,buff,strlen(buff));
			}
		}else{
			printf("distance_U = %f\n",distance_U);
			printf("accel_X = %d\n",accel_X);
			printf("accel_Y = %d\n",accel_Y);
			printf("accel_Z = %d\n",accel_Z);
			printf("gyro_X = %d\n",gyro_X);
			printf("gyro_Y = %d\n",gyro_X);
			printf("gyro_Z = %d\n",gyro_X);

			for (int i=0;i<=11;i++){		
				printf("Sending pwm_values[%d] = %d\n",i,pwm_values[i]);
			}

		}
		printf("\n\n\n");

		ros::spinOnce();
		//ros::Duration(2).sleep();
	}
	return 0;
}
