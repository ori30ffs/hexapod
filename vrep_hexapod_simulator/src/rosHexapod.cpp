#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include "../include/v_repConst.h"
#include "../include/v_repLib.cpp"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include "hexa_msgs/AnglesValues.h"
#include "hexa_msgs/AngleValue.h"
// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/simRosGetObjectPose.h"
#include "vrep_common/simRosSetObjectPose.h"
#include "vrep_common/simRosGetObjectHandle.h"
#include "vrep_common/simRosSetJointState.h"
#include "vrep_common/VrepInfo.h"

#include "vrep_common/JointSetStateData.h"
#include <time.h>
// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include <iostream>
// Global variables (modified by topic subscribers):
bool simulationRunning=true;
bool sensorTrigger=false;
float simulationTime=0.0f;
int baseHandle;
int joint1Handle[6]={0,0,0,0,0,0};
int joint2Handle[6];
int joint3Handle[6];
int sensorHandle;
// Topic subscriber callbacks:
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info)
{
	simulationTime=info->simulationTime.data;
	simulationRunning=(info->simulatorState.data&1)!=0;
}

void sensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens)
{
	printf("sensorCallback\n");
	sensorTrigger=true;
}

ros::Publisher jointsSetPub;

void publisherSend(const hexa_msgs::AnglesValues::ConstPtr& msg)
{
	vrep_common::JointSetStateData jointsSet;
	for(int i=0;i<6;i++)
	{

		if(joint1Handle[i]!=0)
		{
			jointsSet.handles.data.push_back(joint1Handle[i]);
			jointsSet.handles.data.push_back(joint2Handle[i]);
			jointsSet.handles.data.push_back(joint3Handle[i]);
			jointsSet.setModes.data.push_back(0); // 3 force
			jointsSet.setModes.data.push_back(0);
			jointsSet.setModes.data.push_back(0);
			jointsSet.values.data.push_back(msg->Angles[i*3]);
			jointsSet.values.data.push_back(msg->Angles[i*3+1]);
			jointsSet.values.data.push_back(msg->Angles[i*3+2]);
			//isCanPublish=true;
		}
	}
	jointsSetPub.publish(jointsSet);
	//ROS_INFO("gg");
}
void publisherSingleSend(const hexa_msgs::AngleValue::ConstPtr& msg)
{
	vrep_common::JointSetStateData jointsSet;
	if(joint1Handle[msg->legId]!=0)
	{
		jointsSet.handles.data.push_back(joint1Handle[msg->legId]);
		jointsSet.handles.data.push_back(joint2Handle[msg->legId]);
		jointsSet.handles.data.push_back(joint3Handle[msg->legId]);
		jointsSet.setModes.data.push_back(0); // 3 force
		jointsSet.setModes.data.push_back(0);
		jointsSet.setModes.data.push_back(0);
		jointsSet.values.data.push_back(msg->j1);
		jointsSet.values.data.push_back(msg->j2);
		jointsSet.values.data.push_back(msg->j3);
	}
	jointsSetPub.publish(jointsSet);
	//ROS_INFO("gg2");
}

//legs init
bool isInitLeg1=false;
bool isInitLeg2=false;
bool isInitLeg3=false;
bool isInitLeg4=false;
bool isInitLeg5=false;
bool isInitLeg6=false;
//isNotFirstTime?
bool isNotFirstTime1=false;
bool isNotFirstTime2=false;
bool isNotFirstTime3=false;
bool isNotFirstTime4=false;
bool isNotFirstTime5=false;
bool isNotFirstTime6=false;
//strings for get handles
std::string hj1="hexa_joint1";
std::string hj2="hexa_joint2";
std::string hj3="hexa_joint3";

void initLegHandle(std::string sj1,std::string sj2,std::string sj3,int i,ros::NodeHandle node)
{
		ros::ServiceClient getObjHandle_client=node.serviceClient<vrep_common::simRosGetObjectHandle>("/vrep/simRosGetObjectHandle");
		vrep_common::simRosGetObjectHandle legTargetHandle;
		//init j1
		legTargetHandle.request.objectName=sj1;
		getObjHandle_client.call(legTargetHandle);
		joint1Handle[i]=legTargetHandle.response.handle;
		//init j2
		legTargetHandle.request.objectName=sj2;
		getObjHandle_client.call(legTargetHandle);
		joint2Handle[i]=legTargetHandle.response.handle;
		//init j3
		legTargetHandle.request.objectName=sj3;
		getObjHandle_client.call(legTargetHandle);
		joint3Handle[i]=legTargetHandle.response.handle;
}

// Main code:
int main(int argc,char* argv[])
{
	// The joint handles and proximity sensor handles are given in the argument list
	// (when V-REP launches this executable, V-REP will also provide the argument list)

	if (argc>=4)
	{
		baseHandle=atoi(argv[1]);
		joint1Handle[0]=atoi(argv[2]);
		joint2Handle[0]=atoi(argv[3]);
		joint3Handle[0]=atoi(argv[4]);
		sensorHandle=atoi(argv[5]);

		printf("start \n");
	}
	else
	{
		printf("Error'!\n");
		sleep(5000);
		return 0;
	}

	// Create a ROS node. The name has a random component: 
	int _argc = 0;
	char** _argv = NULL;
	struct timeval tv;
	unsigned int timeVal=0;
	if (gettimeofday(&tv,NULL)==0)
		timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
	std::string nodeName("vrep_hexapod_simulator");
	std::string randId(boost::lexical_cast<std::string>(timeVal+int(999999.0f*(rand()/(float)RAND_MAX))));
	nodeName+=randId;		
	ros::init(_argc,_argv,nodeName.c_str());

	if(!ros::master::check())
		return(0);
	
	ros::NodeHandle node("~");	
	printf("rosHexapodLeg just started with node name %s\n",nodeName.c_str());
	ros::Subscriber subInfo=node.subscribe("/vrep/info",1,infoCallback);

	ros::ServiceClient client_enablePublisher=node.serviceClient<vrep_common::simRosEnablePublisher>("/vrep/simRosEnablePublisher");
	vrep_common::simRosEnablePublisher srv_enablePublisher;
	srv_enablePublisher.request.topicName="proxData"+randId; // the requested topic name
	srv_enablePublisher.request.queueSize=1; // the requested publisher queue size (on V-REP side)
	srv_enablePublisher.request.streamCmd=simros_strmcmd_read_proximity_sensor; // the requested publisher type
	srv_enablePublisher.request.auxInt1=sensorHandle; // some additional information the publisher needs (what proximity sensor)

    if ( client_enablePublisher.call(srv_enablePublisher)&&(srv_enablePublisher.response.effectiveTopicName.length()!=0) )
	{ 	// ok, the service call was ok, and the publisher was succesfully started on V-REP side
		// V-REP is now streaming the proximity sensor data!

		printf("publisher\n");
		initLegHandle(hj1,hj2,hj3,0,node);
		initLegHandle(hj1+"#1",hj2+"#1",hj3+"#1",2,node);
		initLegHandle(hj1+"#3",hj2+"#3",hj3+"#3",4,node);

		initLegHandle(hj1+"#0",hj2+"#0",hj3+"#0",1,node);
		initLegHandle(hj1+"#2",hj2+"#2",hj3+"#2",3,node);
		initLegHandle(hj1+"#4",hj2+"#4",hj3+"#4",5,node);
		// 3. Let's subscribe to that proximity sensor data:
		std::string topicName("/vrep/");
		topicName+=srv_enablePublisher.response.effectiveTopicName; // Make sure to use the returned topic name, not the requested one (can be same)
		ros::Subscriber sub=node.subscribe(topicName.c_str(),1,sensorCallback);

		// 4. Let's tell V-REP to subscribe to the motor speed topic (publisher to that topic will be created further down):
		ros::ServiceClient client_enableSubscriber=node.serviceClient<vrep_common::simRosEnableSubscriber>("/vrep/simRosEnableSubscriber");
		vrep_common::simRosEnableSubscriber srv_enableSubscriber;

		srv_enableSubscriber.request.topicName="/"+nodeName+"/legs"; // the topic name
		srv_enableSubscriber.request.queueSize=1; // the subscriber queue size (on V-REP side)
		srv_enableSubscriber.request.streamCmd=simros_strmcmd_set_joint_state; // the subscriber type

		if ( client_enableSubscriber.call(srv_enableSubscriber)&&(srv_enableSubscriber.response.subscriberID!=-1) )
		{	// ok, the service call was ok, and the subscriber was succesfully started on V-REP side
			// V-REP is now listening to the desired motor joint states
			printf("subscriber\n");
			// 5. Let's prepare a publisher of those motor speeds:
			jointsSetPub=node.advertise<vrep_common::JointSetStateData>("legs",1);

			ros::Subscriber kinematicSub=node.subscribe("/vrep/toSimulator",1000, publisherSend);
			ros::Subscriber kinematicSingleSub=node.subscribe("toSimulatorSingle",1000, publisherSingleSend);
		while (ros::ok() && simulationRunning)
			{ 
				ros::spinOnce();
				// sleep a bit:
				usleep(50000);
			}
		}
	}
	ros::shutdown();
	printf("rosHexapodLeg just ended!\n");
	return(0);
}