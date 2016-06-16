#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


#define BAUDRATE 	57600
#define DEVICE 	"/dev/ttyUSB0"  
#define MAXSIZE 30

int ret;
int serial_fd;
int direction;
double velocity;
char read_buf[50];
char ring_buf[MAXSIZE];
int read_addr = 0;
int write_addr = 0;
bool takeoff = false;

std_msgs::Empty order;
geometry_msgs::Twist cmd;

int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) ;
void serial_init();
int read();
int next_data_handle(int addr);
void write_data(char data);//put the data into the ringbuffer;
void read_data();//read the data from ringbuffer to the cmd and velocity;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_control");
	ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
	ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
	ros::Rate loop_rate(50);

	serial_init();
	while(ros::ok())
	{
		cmd.linear.x = 0.0;
		cmd.linear.y = 0.0;
		cmd.linear.z = 0.0;
		cmd.angular.x = 0.0;
		cmd.angular.y = 0.0;
		cmd.angular.z = 0.0;
		ret = read();
        if(ret <= 0)
        {
		    
        }else
		{
			for (int i = 0; i < 6; i++)
			{
				write_data(read_buf[i]);//write the serialread data to ringbuffer
			}
			read_data();//read data from ringbuffer
			ROS_INFO("direction:%d",direction);
			ROS_INFO("velocity:%f",velocity);
			switch(direction){
				case 0:
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					break;
				case 1:
					cmd.linear.x = velocity;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
                    ROS_INFO("linear x %lf",cmd.linear.x);
					break;
				case 2:
					cmd.linear.x = -velocity;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					break;
				case 3:
					cmd.linear.x = 0.0;
					cmd.linear.y = velocity*2;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					break;
				case 4:
					cmd.linear.x = 0.0;
					cmd.linear.y = -velocity;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					break;
				case 5:
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = velocity;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					break;
				case 6:
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = -velocity;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					break;
				case 7:
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = velocity;
					break;
				case 8:
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = -velocity;
					break;
				case 68://takeoff
                    takeoff_pub.publish(order);
					takeoff = true;
					break;
				case 60://land
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					cmd_pub.publish(cmd);
                    land_pub.publish(order);
					takeoff = false;
					break;
				default:
					cmd.linear.x = 0.0;
					cmd.linear.y = 0.0;
					cmd.linear.z = 0.0;
					cmd.angular.x = 0.0;
					cmd.angular.y = 0.0;
					cmd.angular.z = 0.0;
					break;
			}

            if(direction!=68&&direction!=60)cmd_pub.publish(cmd);
		}
        //ROS_INFO("-----linear x %lf",cmd.linear.x);
        //ROS_INFO("PUBLSIH");


		ros::spinOnce();
		loop_rate.sleep();
	}
	close(serial_fd);
	return 0;
}


int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
	struct termios newtio,oldtio;  
	if( tcgetattr( fd,&oldtio)  !=  0) 
	{   
		ROS_INFO("SetupSerial 1");  
		return -1;  
	}  
	bzero( &newtio, sizeof( newtio ) );  
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  

	switch( nBits )  
	{  
		case 7:  
			newtio.c_cflag |= CS7;  
			break;  
		case 8:  
			newtio.c_cflag |= CS8;  
			break;  
	}

	switch( nEvent )  
	{  
		case 'O':  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag |= PARODD;  
			newtio.c_iflag |= (INPCK | ISTRIP);  
			break;  
		case 'E':   
			newtio.c_iflag |= (INPCK | ISTRIP);  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag &= ~PARODD;  
	    	break;
		case 'N':    																									
			newtio.c_cflag &= ~PARENB;  
			break;  
	}  

	switch( nSpeed )  
	{  
		case 2400:  
			cfsetispeed(&newtio, B2400);  
			cfsetospeed(&newtio, B2400);  
			break;  
		case 4800:  
			cfsetispeed(&newtio, B4800);  
			cfsetospeed(&newtio, B4800);  
			break;
		case 9600:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
		case 19200:  
			cfsetispeed(&newtio, B19200);  
			cfsetospeed(&newtio, B19200);  
			break; 
		case 57600:  
			cfsetispeed(&newtio, B57600);  
			cfsetospeed(&newtio, B57600);  
			break; 
		case 115200:  
			cfsetispeed(&newtio, B115200);  
			cfsetospeed(&newtio, B115200);  
			break;  
		case 460800:  
			cfsetispeed(&newtio, B460800);  
			cfsetospeed(&newtio, B460800);  
			break;  
		default:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
	}  

	if( nStop == 1 )  
	{
		newtio.c_cflag &=  ~CSTOPB;  
	}
	else if ( nStop == 2 )
	{  
		newtio.c_cflag |=  CSTOPB;  
	} 

	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
		ROS_INFO("com set error!");  
		return -1;  
	}  
	return 0;  
} 

void serial_init()
{
	serial_fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (serial_fd == -1)
	{
		ROS_INFO("Open Error!");  
		exit(1);    
	}  
	ret = set_serial(serial_fd, BAUDRATE, 8, 'N', 1);
	if (ret == -1)  
	{
		ROS_INFO("Set Serial Error!");  
		exit(1);  
	}
}

int next_data_handle(int addr) {
	return (addr + 1) == MAXSIZE ? 0 : (addr + 1);
}

void write_data(char data)
{
	*(ring_buf+write_addr) = data;
	write_addr = next_data_handle(write_addr);
}

void read_data()
{
	if (ring_buf[read_addr] == 'M')
	{
		direction = ring_buf[read_addr + 1]-48;
        velocity = (ring_buf[read_addr + 4] - 48) / 300.0;
		for (int i = 0; i < 6; i++) {
			read_addr = next_data_handle(read_addr);
		}
	}
	else
	{
		for (int i = 0; i < 6; i++)
		{
			read_addr = next_data_handle(read_addr);
			if (ring_buf[read_addr] == 'M')
			{
				break;
			}
		}
	}
}

int read()
{
	int ret;

	memset(read_buf, 0, 50);
	ret = read(serial_fd, read_buf, 6);
	if (ret < 0)
	{
		ROS_INFO("read error!");
		return -1;
	}
	else if (ret == 0)
	{
		//ROS_INFO("No data!");
		return 0;
	}
	else{
        /*ROS_INFO("data:%c",read_buf[0]);
		ROS_INFO("data:%c",read_buf[1]);
		ROS_INFO("data:%c",read_buf[2]);
		ROS_INFO("data:%c",read_buf[3]);
		ROS_INFO("data:%c",read_buf[4]);
        ROS_INFO("data:%c",read_buf[5]);
        return ret;*/
	}
}
