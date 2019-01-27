/*
Code to get keyboard inputs without blocking the loop and without pressing enter

Found this program here:
https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp
*/

#include <ros/ros.h>
#include <termios.h>
#include "std_msgs/String.h"

char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 1000;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	//else if(rv == 0)
		//ROS_INFO("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "key_input_node");
	ros::NodeHandle n;
	//topic where pressed key gets posted
	ros::Publisher keyboard_input_publisher = n.advertise<std_msgs::String>("keyboard_input", 1000);
	ros::Rate loop_rate(25);
	while (ros::ok())
	{
		int c = 0;
		c=getch();
		std_msgs::String msg;
		ROS_INFO("%c", c);
		std::cerr << "c is " << c << "\n";
		switch(c) 
		{
			//grossesGelenk vor/zureck
			case 113: 
			  msg.data = 'q';
			  break;
			case 97: 
			  msg.data = 'a';
			  break;
			//mittleresGelenk vor/zurueck
			case 119: 
			  msg.data = 'w';
			  break;
			case 115: 
			  msg.data = 's';
			  break;
			//kleinesGelenk vor/zurueck			
			case 101: 
			  msg.data = 'e';
			  break;
			case 100: 
			  msg.data = 'd';
			  break;
			default: break;
		}
		keyboard_input_publisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
