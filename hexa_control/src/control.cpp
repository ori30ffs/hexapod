#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <stdlib.h>
/*
	add msg control
	*/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexa_control");
	ros::NodeHandle n;

	ros::Publisher legsPub =n.advertise<std_msgs::String>("toLegs",10);

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		std_msgs::String msg;
		std::cin>>msg.data;
		if(msg.data=="l" || msg.data=="r")
		{
			ROS_INFO("This msg go to HEXA_LEGS: [%s]",msg.data.c_str());
			legsPub.publish(msg);
		}
		else
		{
			ROS_INFO("INPUT l OR r\n");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	//ros::spin();
	return 0;
}