#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "hexa_msgs/LegsJointsStateSrv.h"
#include "hexa_msgs/AnglesValues.h"
#include "hexa_msgs/AngleValue.h"
#include <kdl/frames.hpp>

ros::ServiceClient client;
ros::Publisher jointsSetPub;

bool simulation=false;

float j1_value[6]={0,0,0,0,0,0};
float j2_value[6]={-30,-30,-30,-30,-30,-30};
float j3_value[6]={90,90,90,90,90,90};

bool isCanUp1[6]={true,true,true,true,true,true};
//bool isCanUp1[6]={true,true,true,false,false,false};
bool isCanUp2[6]={true,true,true,true,true,true};

float PI=3.14159265359;

void publisherSend()
{
	hexa_msgs::AnglesValues jointsSet;
	for(int i=0;i<6;i++)
	{
			jointsSet.Angles.push_back(j1_value[i]*PI/180);
			jointsSet.Angles.push_back((j2_value[i])*PI/180);
			jointsSet.Angles.push_back((-j2_value[i]+j3_value[i])*PI/180);
	}
	jointsSetPub.publish(jointsSet);
}

void controlCallback(const std_msgs::String::ConstPtr& msg)
{
	if(simulation==true)
	{
		ROS_INFO("SIMULATION Body END NOW\n");
		simulation=false; 
	}
	else 
	{
		simulation=true;
		ROS_INFO("SIMULATION Body START NOW\n");
	}
}

//all up or down joint2
void updateJoint2(int i)
{
	//if(i!=1 && i!=2 && i!=3)
	if(isCanUp2[i])
	{
		if(j2_value[i]<=45)
			j2_value[i]+=0.001;
		else
			isCanUp2[i]=false;
		
	}
	else
	{
		if(j2_value[i]>-40)
			j2_value[i]-=0.001;
		else
			isCanUp2[i]=true;
	}		
}
//all move joint1
void updateJoint1(int i)
{
	//if(i!=1 && i!=2 && i!=3)
	if(isCanUp1[i])
	{
		if(j1_value[i]<=45)
			j1_value[i]+=0.001;
		else
			isCanUp1[i]=false;
		
	}
	else
	{
		if(j1_value[i]>-45)
			j1_value[i]-=0.001;
		else
			isCanUp1[i]=true;
	}		
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexa_kinematics_legs_client");
	ros::NodeHandle n;

	ros::Subscriber controlSub=n.subscribe("toBody",10, controlCallback);
	client =n.serviceClient<hexa_msgs::LegsJointsStateSrv>("hexa_kinematics");
	jointsSetPub=n.advertise<hexa_msgs::AnglesValues>("/vrep/toSimulator",1000);
	while(ros::ok() /*&& simulation*/)
	{
		for(int i=0;i<6;i++)
			updateJoint1(i);
		publisherSend();
	}
	ros::spin();
	return 0;
}