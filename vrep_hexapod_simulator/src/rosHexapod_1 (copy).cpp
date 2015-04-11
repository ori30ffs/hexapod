#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <ros/ros.h>
#include "../include/v_repConst.h"
#include "../include/v_repLib.cpp"
#include <string>
#include <geometry_msgs/PoseStamped.h>
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
	//printf("infoCallback\n");
}

void sensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens)
{
	// We don't care about the detected distance here, we just trigger!
	printf("sensorCallback\n");
	sensorTrigger=true;
}

ros::Publisher jointsSetPub;

bool isCanUp3[6]={true,true,true,true,true,true};
bool isCanUp2[6]={true,true,true,true,true,true};
bool isCanUp1[6]={true,true,true,true,true,true};
float j1_value[6]={0,0,0,0,0,0};
float j2_value[6]={-30,-30,-30,-30,-30,-30};
float j3_value[6]={90,90,90,90,90,90};
float PI=3.14159265359;
bool isBlock[6]={true,false,true,false,true,false};
bool isBlockFlag=true;

void publisherSend()
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
			jointsSet.values.data.push_back(j1_value[i]*PI/180);
			jointsSet.values.data.push_back((j2_value[i])*PI/180);
			jointsSet.values.data.push_back((-j2_value[i]+j3_value[i])*PI/180);
			//isCanPublish=true;
		}
	}
		jointsSetPub.publish(jointsSet);
}

void simpleKinematics(int i)
{
				if(isCanUp3[i])
				{
					if(j3_value[i]<=90)
						j3_value[i]+=1;
					else
					{
						isCanUp3[i]=false;
						if(i==0)
							if(isBlockFlag)isBlockFlag=false;
						if(i==1)
							if(!isBlockFlag)isBlockFlag=true;
					}
				}
				else
				{
					if(j3_value[i]>-20)
						j3_value[i]-=1;
					else
						isCanUp3[i]=true;
				}
				if(isCanUp2[i])
				{
					if(j2_value[i]>-75)
						j2_value[i]-=0.5;
					else
						isCanUp2[i]=false;
				}
				else
				{
					if(j2_value[i]<=-30)
						j2_value[i]+=0.5;
					else
						isCanUp2[i]=true;
				}		
				if(isCanUp1[i])
				{
					if(j1_value[i]<=45)
						j1_value[i]+=0.5;
					else
						isCanUp1[i]=false;
				}
				else
				{
					if(j1_value[i]>-45)
						j1_value[i]-=0.5;
					else
						isCanUp1[i]=true;
				}	
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

void updateLegUniversal(int legID)
{
	bool isInitLegs;
	bool isNotFirstTime;
	switch(legID)
	{
		case 1:
			isInitLegs=isInitLeg1;
			isNotFirstTime=isNotFirstTime1;
			break;
		case 2:
			isInitLegs=isInitLeg2;
			isNotFirstTime=isNotFirstTime2;
			break;
		case 3:
			isInitLegs=isInitLeg3;
			isNotFirstTime=isNotFirstTime3;
			break;
		case 4:
			isInitLegs=isInitLeg4;
			isNotFirstTime=isNotFirstTime4;
			break;
		case 5:
			isInitLegs=isInitLeg5;
			isNotFirstTime=isNotFirstTime5;
			break;
		case 6:
			isInitLegs=isInitLeg6;
			isNotFirstTime=isNotFirstTime6;
			break;
	}
	if(!isInitLegs)
	{
		vrep_common::JointSetStateData jointPos;
		jointPos.handles.data.push_back(joint1Handle[legID]);
		jointPos.handles.data.push_back(joint2Handle[legID]);
		jointPos.handles.data.push_back(joint3Handle[legID]);
		jointPos.setModes.data.push_back(0); // 3 force
		jointPos.setModes.data.push_back(0);
		jointPos.setModes.data.push_back(0);
		jointPos.values.data.push_back(0);
		jointPos.values.data.push_back(-30*3.14/180);
		jointPos.values.data.push_back(120*3.14/180);
		jointsSetPub.publish(jointPos);

		switch(legID)
		{
			case 1:
				isInitLeg1=true;
				break;
			case 2:
				isInitLeg2=true;
				break;
			case 3:
				isInitLeg3=true;
				break;
			case 4:
				isInitLeg4=true;
				break;
			case 5:
				isInitLeg5=true;
				break;
			case 6:
				isInitLeg6=true;
				break;
		}
	}
	else
	{
		if(!isNotFirstTime)
		{
			switch(legID)
			{
				case 1:
					isNotFirstTime1=true;
					break;
				case 2:
					isNotFirstTime2=true;
					break;
				case 3:
					isNotFirstTime3=true;
					break;
				case 4:
					isNotFirstTime4=true;
					break;
				case 5:
					isNotFirstTime5=true;
					break;
				case 6:
					isNotFirstTime6=true;
					break;
			}		
		}
		else
		{
			if(isBlock[legID-1])
				simpleKinematics(legID-1);
		}
	}
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
	std::string nodeName("rosHexLeg");
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
		printf("publisher\n");
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

			while (ros::ok() && simulationRunning)
			{ 
				/*isBlock[0]=true;
				isBlock[1]=true;
				isBlock[2]=true;
				isBlock[3]=true;
				isBlock[4]=true;
				isBlock[5]=true;*/
				if(isBlockFlag)
				{
					isBlock[0]=true;
					isBlock[1]=false;
					isBlock[2]=true;
					isBlock[3]=false;
					isBlock[4]=true;
					isBlock[5]=false;
				}
				else
				{
					isBlock[0]=false;
					isBlock[1]=true;
					isBlock[2]=false;
					isBlock[3]=true;
					isBlock[4]=false;
					isBlock[5]=true;
				}
				updateLegUniversal(1);
				updateLegUniversal(3);
				updateLegUniversal(5);
				//2
				updateLegUniversal(2);
				updateLegUniversal(4);
				updateLegUniversal(6);
				publisherSend();
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