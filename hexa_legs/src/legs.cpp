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

bool isCanUp3[6]={true,true,true,true,true,true};
bool isCanUp2[6]={true,true,true,true,true,true};
bool isCanUp1[6]={true,true,true,true,true,true};
float j1_value[6]={0,5,10,0,5,10};
float j2_value[6]={-30,-40,-30,-30,-30,-40};
float j3_value[6]={110,90,90,70,90,90};
float PI=3.14159265359;
bool isBlock[6]={true,false,true,false,true,false};
bool isBlockFlag=true;

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


ros::Publisher jointsSetPub;
ros::Publisher jointsSetSinglePub;

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

void step5plus1(int i);
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
		 
		hexa_msgs::AngleValue jointPos;//do angle value
		jointPos.j1=0;
		jointPos.j2=-30*3.14/180;
		jointPos.j3=120*3.14/180;
		jointPos.legId=legID-1;
		jointsSetSinglePub.publish(jointPos);

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
			//if(isBlock[legID-1])
				//simpleKinematics(legID-1);
			step5plus1(legID-1);
		}
	}
}

int kinematicsModes[6]={1,5,6,4,5,6};//1 5 3 4 2 6

void step5plus1(int i)
{
	switch(kinematicsModes[i])
	{
		case 1:
		{
			if(isCanUp3[i])
			{
				if(j3_value[i]<=110)
					j3_value[i]+=0.4;
				else
					isCanUp3[i]=false;
			}
			else
			{
				if(j3_value[i]>70)
					j3_value[i]-=0.4;
				else
					isCanUp3[i]=true;
			}
			if(isCanUp2[i])
			{
				if(j2_value[i]>-40)
					j2_value[i]-=0.2;
				else
					isCanUp2[i]=false;
			}
			else
			{
				if(j2_value[i]<=-20)
					j2_value[i]+=0.2;
				else
					isCanUp2[i]=true;
			}		
			break;
		}
		case 2:
		{
			if(isCanUp2[i])
			{
				if(j2_value[i]>-40)
					j2_value[i]-=0.2;
				else
					isCanUp2[i]=false;
			}
			else
			{
				if(j2_value[i]<=-20)
					j2_value[i]+=0.2;
				else
					isCanUp2[i]=true;
			}			
			if(isCanUp1[i])
			{
				if(j1_value[i]>-30)
					j1_value[i]-=0.4;
				else
					isCanUp1[i]=false;
			}
			else
			{
				if(j1_value[i]<10)
					j1_value[i]+=0.4;
				else
					isCanUp1[i]=true;
			}	
			break;
		}
		case 3:
		{
			if(isCanUp3[i])
			{
				if(j3_value[i]<=90)
					j3_value[i]+=0.3;
				else
					isCanUp3[i]=false;
			}
			else
			{
				if(j3_value[i]>70)
					j3_value[i]-=0.3;
				else
					isCanUp3[i]=true;
			}
			if(isCanUp2[i])
			{
				if(j2_value[i]>-30)
					j2_value[i]-=0.1;
				else
					isCanUp2[i]=false;
			}
			else
			{
				if(j2_value[i]<=-20)
					j2_value[i]+=0.1;
				else
					isCanUp2[i]=true;
			}			
			if(isCanUp1[i])
			{
				if(j1_value[i]<=35)
					j1_value[i]+=0.35;
				else
					isCanUp1[i]=false;
			}
			else
			{
				if(j1_value[i]>0)
					j1_value[i]-=0.35;
				else
					isCanUp1[i]=true;
			}	
			break;
	}
	case 4:
		{
			if(isCanUp3[i])
			{
				if(j3_value[i]<=110)
					j3_value[i]+=0.4;
				else
					isCanUp3[i]=false;
			}
			else
			{
				if(j3_value[i]>70)
					j3_value[i]-=0.4;
				else
					isCanUp3[i]=true;
			}
			if(isCanUp2[i])
			{
				if(j2_value[i]>-40)
					j2_value[i]-=0.2;
				else
					isCanUp2[i]=false;
			}
			else
			{
				if(j2_value[i]<=-20)
					j2_value[i]+=0.2;
				else
					isCanUp2[i]=true;
			}		
			break;
	}
	case 5:
		{
			
			if(isCanUp2[i])
			{
				if(j2_value[i]>-40)
					j2_value[i]-=0.2;
				else
					isCanUp2[i]=false;
			}
			else
			{
				if(j2_value[i]<=-20)
					j2_value[i]+=0.2;
				else
					isCanUp2[i]=true;
			}			
			if(isCanUp1[i])
			{
				if(j1_value[i]<=30)
					j1_value[i]+=0.4;
				else
					isCanUp1[i]=false;
			}
			else
			{
				if(j1_value[i]>-10)
					j1_value[i]-=0.4;
				else
					isCanUp1[i]=true;
			}	
			break;
		}
	case 6:
		{
			
			if(isCanUp2[i])
			{
				if(j2_value[i]>-40)
					j2_value[i]-=0.2;
				else
					isCanUp2[i]=false;
			}
			else
			{
				if(j2_value[i]<=-20)
					j2_value[i]+=0.2;
				else
					isCanUp2[i]=true;
			}			
			if(isCanUp1[i])
			{
				if(j1_value[i]>-30)
					j1_value[i]-=0.4;
				else
					isCanUp1[i]=false;
			}
			else
			{
				if(j1_value[i]<10)
					j1_value[i]+=0.4;
				else
					isCanUp1[i]=true;
			}	
			break;
		}
	}
}

bool simulation=false;
void guit()
{
	while(ros::ok() && simulation)
	{
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
			//ROS_INFO("From");
			usleep(5000);
	}
}
void controlCallback(const std_msgs::String::ConstPtr& msg)
{
	simulation=true;
	ROS_INFO("SIMULATION GUIT START NOW\n");
	guit();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexa_kinematics_legs_client");
	ros::NodeHandle n;
	KDL::Vector v1;
	ros::Subscriber controlSub=n.subscribe("toLegs",10, controlCallback);
	//Client-Server--------------------------------------------
	client =n.serviceClient<hexa_msgs::LegsJointsStateSrv>("hexa_kinematics");
	jointsSetPub=n.advertise<hexa_msgs::AnglesValues>("/vrep/toSimulator",1000);
	jointsSetSinglePub=n.advertise<hexa_msgs::AngleValue>("toSimulatorSingle",1000);
	//End-Client-Server--------------------------------------------
	
	ros::spin();
	return 0;
}