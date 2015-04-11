#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include "ros/ros.h"
#include "hexa_msgs/LegsJointsStateSrv.h"
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int64.h>
#include "hexa_msgs/AnglesValues.h"
//#include <gtk.h>
float PI=3.1415926535897932384626433832;

ros::Publisher jointsSetPub;
float j1_value[6]={0,0,0,0,0,10};
float j2_value[6]={-30,-30,-30,-30,-30,-30};
float j3_value[6]={90,90,90,90,90,90};
void publisherSend(float coxa[6],float femur[6],float tibia[6])
{
	hexa_msgs::AnglesValues jointsSet;
	for(int i=0;i<6;i++)
	{
			jointsSet.Angles.push_back((j1_value[i]+coxa[i])*PI/180);
			jointsSet.Angles.push_back((j2_value[i]+femur[i])*PI/180);
			jointsSet.Angles.push_back((-j2_value[i]+j3_value[i]+tibia[i])*PI/180);
	}
	jointsSetPub.publish(jointsSet);
}

void spreadsheet(int PosX=0, int PosY=0,int PosZ=0,int RotX=0,int RotY=0,int RotZ=0)
{
	float sideLength =55;
	float coxaLength=30;
	float femurLength=60;
	float tibiaLength=70;

	float bodyCoxaOffsetX=sideLength/2;
	float bodyCoxaOffsetZ=sqrt(sideLength*sideLength-bodyCoxaOffsetX*bodyCoxaOffsetX);
	float bodyCoxaOffset=sideLength;

	float X[6]={bodyCoxaOffsetX,bodyCoxaOffset,bodyCoxaOffsetX,-bodyCoxaOffsetX,-bodyCoxaOffset,-bodyCoxaOffsetX};
	float Z[6]={bodyCoxaOffsetZ,0,-bodyCoxaOffsetZ,-bodyCoxaOffsetZ,0, bodyCoxaOffsetZ};

	float feetPosXcoxa[6]={cos(60./180*PI)*(coxaLength+femurLength),coxaLength+femurLength,
						   cos(60./180*PI)*(coxaLength+femurLength),-cos(60./180*PI)*(coxaLength+femurLength),
						   -coxaLength-femurLength,-cos(60./180*PI)*(coxaLength+femurLength)};
	float feetPosYcoxa[6]={tibiaLength,tibiaLength,tibiaLength,tibiaLength,tibiaLength,tibiaLength};
	float feetPosZcoxa[6]={sin(60./180*PI)*(coxaLength+femurLength),0,-sin(60./180*PI)*(coxaLength+femurLength),
						   -sin(60./180*PI)*(coxaLength+femurLength),0,sin(60./180*PI)*(coxaLength+femurLength)};

	/*float X1[6]={X[5],X[0],X[1],X[2],X[3],X[4]};
	float Z1[6]={Z[5],Z[0],Z[1],Z[2],Z[3],Z[4]};
	float X2[6]={X[0],X[1],X[2],X[3],X[4],X[5]};
	float Z2[6]={Z[0],Z[1],Z[2],Z[3],Z[4],Z[5]};*/
	//for(int i=0;i<6;i++)
		//printf(" pos: 1 %lf 2 %lf 3 %lf \n",feetPosXcoxa[i],feetPosYcoxa[i],feetPosZcoxa[i]);
	//bodyIK
	float TotalZ[6];
	float TotalX[6];
	float DistBodyCenterFeet[6];
	float AngleBodyCenterX[6];
	float RollY[6];
	float PitchY[6];
	float BodyIkX[6];
	float BodyIkZ[6];
	float BodyIkY[6];

	for(int i=0;i<6;i++)
	{
		TotalZ[i]=feetPosZcoxa[i]+Z[i]+PosZ;
		TotalX[i]=feetPosXcoxa[i]+X[i]+PosX;
		//printf(" pos: 1 %lf 2 %lf \n",TotalZ[i],TotalX[i]);
		DistBodyCenterFeet[i]=sqrt(TotalZ[i]*TotalZ[i]+TotalX[i]*TotalX[i]);
		AngleBodyCenterX[i]=PI/2-atan2f(TotalX[i],TotalZ[i]);
		//printf(" coxadist: 1 %lf \n",AngleBodyCenterX[i]);
		RollY[i]=tan(RotZ*PI/180)*TotalX[i];
		PitchY[i]=tan(RotX*PI/180)*TotalZ[i];
		BodyIkX[i]=cos(AngleBodyCenterX[i]+(RotY*PI/180))*DistBodyCenterFeet[i]-TotalX[i];
		BodyIkZ[i]=sin(AngleBodyCenterX[i]+(RotY*PI/180))*DistBodyCenterFeet[i]-TotalZ[i];
		BodyIkY[i]=RollY[i]+PitchY[i];
		//printf(" pos: 1 %lf 2 %lf 3 %lf \n",BodyIkX[i],BodyIkY[i],BodyIkZ[i]);
	}

	//legIK
	float newPosX[6];
	float newPosY[6];
	float newPosZ[6];
	float coxaFeetDist[6];
	float IKSW[6];
	float IKA1[6];
	float IKA2[6];
	float TAngle[6];

	float IKTibiaAngle[6];
	float IKFemurAngle[6];
	float IKCoxaAngle[6];

	for(int i=0;i<6;i++)
	{
		newPosX[i]=feetPosXcoxa[i]+PosX+BodyIkX[i];
		newPosY[i]=feetPosYcoxa[i]+PosY+BodyIkY[i];
		newPosZ[i]=feetPosZcoxa[i]+PosZ+BodyIkZ[i];
			//printf(" pos: 1 %lf 2 %lf 3 %lf \n",newPosX[i],newPosY[i],newPosZ[i]);
		coxaFeetDist[i]=sqrt(newPosX[i]*newPosX[i]+newPosZ[i]*newPosZ[i]);
			//printf(" coxadist: 1 %lf \n",coxaFeetDist[i]);
		IKSW[i]=sqrt((coxaFeetDist[i]-coxaLength)*(coxaFeetDist[i]-coxaLength)+newPosY[i]*newPosY[i]);
		IKA1[i]=atan((coxaFeetDist[i]-coxaLength)/newPosY[i]);
		IKA2[i]=acos((tibiaLength*tibiaLength-femurLength*femurLength-IKSW[i]*IKSW[i])/(-2*IKSW[i]*femurLength));
			printf(" ik: 1 %lf 2 %lf 3 %lf \n",IKSW[i],IKA1[i],IKA2[i]);
		TAngle[i]=acos((IKSW[i]*IKSW[i]-tibiaLength*tibiaLength-femurLength*femurLength)/(-2*femurLength*tibiaLength));
		IKTibiaAngle[i]=90-TAngle[i]*180/PI;
		IKFemurAngle[i]=90-(IKA1[i]+IKA2[i])*180/PI;
		IKCoxaAngle[i]=90-atan2(newPosX[i],newPosZ[i])*180/PI;
	}
	//legs angles
	float coxa[6]={IKCoxaAngle[0]-60,IKCoxaAngle[1],IKCoxaAngle[2]+60,IKCoxaAngle[3]-60-180,IKCoxaAngle[4]-180,IKCoxaAngle[5]+60-180};
	float femur[6];
	float tibia[6];
	for(int i=0;i<6;i++)
	{
		femur[i]=IKFemurAngle[i];
		tibia[i]=IKTibiaAngle[i];
	}
	for(int i=0;i<6;i++)
	{
		printf("LEG [%d] angles: coxa %lf femur %lf tibia %lf \n",i,coxa[i],femur[i],tibia[i]);
	}
	publisherSend(coxa,femur,tibia);
}


const float cos30=cos(0.5236);
const float sin30=sin(0.5236);

const float cos90=cos(PI/2);
const float sin90=sin(PI/2);

const float cos150=cos(2.618);
const float sin150=sin(2.618);

const float cos210=cos(3.6652);
const float sin210=sin(3.6652); 

const float cos270=cos(PI/2+PI);
const float sin270=sin(PI/2+PI);

const float cos330=cos(5.7596);
const float sin330=sin(5.7596);



int legNrTripod[6]={1,4,1,4,1,4};
int legNrWave[6]={13,16,1,4,7,10};
int legNrRipple[6]={7,10,1,7,4,1};

int legsIndex[6]={0,1,2,3,4,5};
int isBodyMoves=false;
int rotate=0;
bool simulation=false;

int GPosX=0;
int GPosY=0;
int GPosZ=0;
int GRotX=0;
int GRotY=0;
int GRotZ=0;

int gait=1;

void waveGait(int step, float* gaitPosX, float* gaitPosZ, float* gaitPosY, float* gaitRotY)
{
	float localLift=10;
	float localMoveX=25;
	float localMoveZ=25;

	float legLiftHeight;
	float travelLengthX;
	float travelLengthZ;
	float travelRotationY=0;
	/**
	* convert value with angle
	*/
	switch(rotate)
	{
		case 0:
		{
			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=localMoveZ;
			break;
		}
		case 1:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=localMoveZ/2;
			break;
		}	
		case 2:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 3:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX/2;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 4:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 5:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=-localMoveZ/2;
			break;
		}
		case 6:
		{
			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=localMoveZ;
			break;
		}
		case 7:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX/2;
			travelLengthZ=localMoveZ;
			break;
		}
		default:break;
	}

	for(int i=0;i<6;i++)
	{
		legNrWave[i]++;
		legNrWave[i]=legNrWave[i]%18;
		if(legNrWave[i]==1)
		{
			gaitPosX[legsIndex[i]]=0; //-travelLengthX;
			gaitPosZ[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=legLiftHeight;
			gaitRotY[legsIndex[i]]=0;
		}
		else if(legNrWave[i]==2)
		{
			gaitPosX[legsIndex[i]]=travelLengthX;
			gaitPosZ[legsIndex[i]]=travelLengthZ;
			gaitPosY[legsIndex[i]]=legLiftHeight;
			gaitRotY[legsIndex[i]]=0;
		}
		else if(legNrWave[i]==3)
		{
			gaitPosX[legsIndex[i]]=0;
			gaitPosZ[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=0;
			gaitRotY[legsIndex[i]]=0;
		}	
		else
		{
			gaitPosX[legsIndex[i]]=0;
			gaitPosZ[legsIndex[i]]=0;
			gaitRotY[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=0;
		}
	}
}

void rippleGait(int step, float* gaitPosX, float* gaitPosZ, float* gaitPosY, float* gaitRotY)
{
	float localLift=10;
	float localMoveX=25;
	float localMoveZ=25;

	float legLiftHeight;
	float travelLengthX;
	float travelLengthZ;
	float travelRotationY=0;
	/**
	* convert value with angle
	*/
	switch(rotate)
	{
		case 0:
		{
			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=localMoveZ;
			break;
		}
		case 1:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=localMoveZ/2;
			break;
		}	
		case 2:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 3:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX/2;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 4:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 5:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=-localMoveZ/2;
			break;
		}
		case 6:
		{
			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=localMoveZ;
			break;
		}
		case 7:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX/2;
			travelLengthZ=localMoveZ;
			break;
		}
		default:break;
	}

	for(int i=0;i<6;i++)
	{
		legNrRipple[i]++;
		legNrRipple[i]=legNrRipple[i]%12;
		if(legNrRipple[i]==1)
		{
			gaitPosX[legsIndex[i]]=0; //-travelLengthX;
			gaitPosZ[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=legLiftHeight;
			gaitRotY[legsIndex[i]]=0;
		}
		else if(legNrRipple[i]==2)
		{
			gaitPosX[legsIndex[i]]=travelLengthX;
			gaitPosZ[legsIndex[i]]=travelLengthZ;
			gaitPosY[legsIndex[i]]=legLiftHeight;
			gaitRotY[legsIndex[i]]=0;
		}
		else if(legNrRipple[i]==3)
		{
			gaitPosX[legsIndex[i]]=0;
			gaitPosZ[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=0;
			gaitRotY[legsIndex[i]]=0;
		}	
		else
		{
			gaitPosX[legsIndex[i]]=0;
			gaitPosZ[legsIndex[i]]=0;
			gaitRotY[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=0;
		}
	}
}

void tripodGait(int step, float* gaitPosX, float* gaitPosZ, float* gaitPosY, float* gaitRotY)
{
	float localLift=10;
	float localMoveX=25;
	float localMoveZ=25;
	float hypotenuse=sqrt(1250);

	float legLiftHeight;
	float travelLengthX;
	float travelLengthZ;
	float travelRotationY=0;
	/**
	* convert value with angle
	*/
	switch(rotate)
	{
		case 0:
		{
			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=localMoveZ;
			break;
		}
		case 1:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=localMoveZ/2;
			break;
		}	
		case 2:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 3:
		{

			legLiftHeight=localLift;
			travelLengthX=localMoveX/2;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 4:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=-localMoveZ;
			break;
		}
		case 5:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=-localMoveZ/2;
			break;
		}
		case 6:
		{
			legLiftHeight=localLift;
			travelLengthX=-localMoveX;
			travelLengthZ=localMoveZ;
			break;
		}
		case 7:
		{

			legLiftHeight=localLift;
			travelLengthX=-localMoveX/2;
			travelLengthZ=localMoveZ;
			break;
		}
		default:break;
	}
	for(int i=0;i<6;i++)
	{
		legNrTripod[i]++;
		legNrTripod[i]=legNrTripod[i]%6;
		if(legNrTripod[i]==1)
		{
			gaitPosX[legsIndex[i]]=0; //-travelLengthX;
			gaitPosZ[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=legLiftHeight;
			gaitRotY[legsIndex[i]]=0;
		}
		else if(legNrTripod[i]==2)
		{
			gaitPosX[legsIndex[i]]=travelLengthX;
			gaitPosZ[legsIndex[i]]=travelLengthZ;
			gaitPosY[legsIndex[i]]=legLiftHeight;
			gaitRotY[legsIndex[i]]=0;
		}
		else if(legNrTripod[i]==3)
		{
			gaitPosX[legsIndex[i]]=0;
			gaitPosZ[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=0;
			gaitRotY[legsIndex[i]]=0;
		}	
		else
		{
			gaitPosX[legsIndex[i]]=0;
			gaitPosZ[legsIndex[i]]=0;
			gaitRotY[legsIndex[i]]=0;
			gaitPosY[legsIndex[i]]=0;
		}
	}
}
void inverseKinematics(int PosX=0, int PosY=0,int PosZ=0,int RotX=0,int RotY=0,int RotZ=0,int step=0)
{
	float cur[6]={1,1,1,1,1,1};

	float sideLength =20;
	float coxaLength=12;
	float femurLength=30;
	float tibiaLength=38;

	float gaitPosX[6]={0,0,0,0,0,0};
	float gaitPosY[6]={0,0,0,0,0,0};
	float gaitPosZ[6]={0,0,0,0,0,0};
	float gaitRotY[6]={0,0,0,0,0,0};
	//isBodyMoves=true;
	if(step!=0)
	{
		switch(gait)
		{
			case 1:
				tripodGait(step,gaitPosX,gaitPosZ,gaitPosY,gaitRotY);
				break;
			case 2:
				waveGait(step,gaitPosX,gaitPosZ,gaitPosY,gaitRotY);
				break;
			case 3:
				rippleGait(step,gaitPosX,gaitPosZ,gaitPosY,gaitRotY);
				break;
			default: break;
		}
	}

	//-----------------end-gait----------------------
	float bodyCoxaOffsetX=sideLength/2;
	float bodyCoxaOffsetZ=sqrt(sideLength*sideLength-bodyCoxaOffsetX*bodyCoxaOffsetX);
	float bodyCoxaOffset=sideLength;

	float X[6]={bodyCoxaOffsetX,bodyCoxaOffset,bodyCoxaOffsetX,-bodyCoxaOffsetX,-bodyCoxaOffset,-bodyCoxaOffsetX};
	float Z[6]={bodyCoxaOffsetZ,0,-bodyCoxaOffsetZ,-bodyCoxaOffsetZ,0, bodyCoxaOffsetZ};

	float feetPosXcoxa[6]={cos(60./180*PI)*(coxaLength+femurLength),coxaLength+femurLength,
						   cos(60./180*PI)*(coxaLength+femurLength),-cos(60./180*PI)*(coxaLength+femurLength),
						   -coxaLength-femurLength,-cos(60./180*PI)*(coxaLength+femurLength)};
	float feetPosYcoxa[6]={tibiaLength,tibiaLength,tibiaLength,tibiaLength,tibiaLength,tibiaLength};
	float feetPosZcoxa[6]={sin(60./180*PI)*(coxaLength+femurLength),0,-sin(60./180*PI)*(coxaLength+femurLength),
						   -sin(60./180*PI)*(coxaLength+femurLength),0,sin(60./180*PI)*(coxaLength+femurLength)};

	//bodyIK
	float TotalZ[6]={gaitPosZ[legsIndex[0]],gaitPosZ[legsIndex[1]],gaitPosZ[legsIndex[2]],gaitPosZ[legsIndex[3]],gaitPosZ[legsIndex[4]],gaitPosZ[legsIndex[5]]};
	float TotalX[6]={gaitPosX[legsIndex[0]],gaitPosX[legsIndex[1]],gaitPosX[legsIndex[2]],gaitPosX[legsIndex[3]],gaitPosX[legsIndex[4]],gaitPosX[legsIndex[5]]};
	float TotalY[6]={gaitPosY[legsIndex[0]],gaitPosY[legsIndex[1]],gaitPosY[legsIndex[2]],gaitPosY[legsIndex[3]],gaitPosY[legsIndex[4]],gaitPosY[legsIndex[5]]};

	float sinRotX=sin(RotX*PI/180);
	float cosRotX=cos(RotX*PI/180);
	float sinRotZ=sin(RotZ*PI/180);
	float cosRotZ=cos(RotZ*PI/180);
	float sinRotY=sin(RotY*PI/180);
	float cosRotY=cos(RotY*PI/180);

	float BodyIkX[6];
	float BodyIkZ[6];
	float BodyIkY[6];
	printf("\n");
	for(int i=0;i<6;i++)
	{
		TotalZ[i]=TotalZ[i]+feetPosZcoxa[i]+Z[i]+PosZ;
		TotalX[i]=TotalX[i]+feetPosXcoxa[i]+X[i]+PosX;
		TotalY[i]=TotalY[i]+feetPosYcoxa[i];
		//printf("1 [%d] : %lf %lf  %lf \n",i,TotalX[i],TotalZ[i],TotalY[i]);
		BodyIkX[i]=TotalX[i]*cosRotZ*cosRotY-TotalZ[i]*cosRotZ*sinRotY+TotalY[i]*sinRotZ-TotalX[i];
		BodyIkZ[i]=(TotalX[i]*cosRotX*sinRotY+TotalX[i]*cosRotY*sinRotZ*sinRotX+TotalZ[i]*cosRotY*cosRotX-TotalZ[i]*sinRotY*sinRotZ*sinRotX-TotalY[i]*cosRotZ*sinRotX)-TotalZ[i];
		BodyIkY[i]=(TotalX[i]*sinRotY*sinRotX-TotalX[i]*cosRotY*cosRotX*sinRotZ+TotalZ[i]*cosRotY*sinRotX+TotalZ[i]*cosRotX*sinRotY*sinRotZ+TotalY[i]*cosRotZ*cosRotX)-TotalY[i];
	}

	//legIK
	float newPosX[6];
	float newPosY[6];
	float newPosZ[6];
	float coxaFeetDist[6];
	float IKSW[6];
	float IKA1[6];
	float IKA2[6];
	float TAngle[6];

	float IKTibiaAngle[6];
	float IKFemurAngle[6];
	float IKCoxaAngle[6];

	for(int i=0;i<6;i++)
	{
		newPosX[i]=gaitPosX[i]+feetPosXcoxa[i]+PosX+BodyIkX[i];
		newPosY[i]=gaitPosY[i]+feetPosYcoxa[i]+PosY+BodyIkY[i];
		newPosZ[i]=gaitPosZ[i]+feetPosZcoxa[i]+PosZ+BodyIkZ[i];
			//printf(" pos: 1 %lf 2 %lf 3 %lf \n",newPosX[i],newPosY[i],newPosZ[i]);
		coxaFeetDist[i]=sqrt(newPosX[i]*newPosX[i]+newPosZ[i]*newPosZ[i]);
			//printf(" coxadist: 1 %lf \n",coxaFeetDist[i]);
		IKSW[i]=sqrt((coxaFeetDist[i]-coxaLength)*(coxaFeetDist[i]-coxaLength)+newPosY[i]*newPosY[i]);
		IKA1[i]=atan((coxaFeetDist[i]-coxaLength)/newPosY[i]);
		IKA2[i]=acos((tibiaLength*tibiaLength-femurLength*femurLength-IKSW[i]*IKSW[i])/(-2*IKSW[i]*femurLength));
			//printf(" ik: 1 %lf 2 %lf 3 %lf \n",IKSW[i],IKA1[i],IKA2[i]);
		TAngle[i]=acos((IKSW[i]*IKSW[i]-tibiaLength*tibiaLength-femurLength*femurLength)/(-2*femurLength*tibiaLength));
		IKTibiaAngle[i]=90-TAngle[i]*180/PI;
		IKFemurAngle[i]=-90+(IKA1[i]+IKA2[i])*180/PI;
		IKCoxaAngle[i]=90-atan2(newPosX[i]*sin((i*60+30.)/180*PI)+newPosZ[i]*cos((i*60+30.)/180*PI),newPosX[i]*cos((i*60+30.)/180*PI)-newPosZ[i]*sin((i*60+30.)/180*PI))*180./PI;
		printf(" ik: 1 %lf 2 %lf 3 %lf \n",IKTibiaAngle[i],IKFemurAngle[i],IKCoxaAngle[i]);
	}
	//legs angles
	float coxa[6];
	float femur[6];
	float tibia[6];
	for(int i=0;i<6;i++)
	{
		coxa[i]=IKCoxaAngle[i];
		femur[i]=IKFemurAngle[i];
		tibia[i]=IKTibiaAngle[i];
	}
	for(int i=0;i<6;i++)
	{
		printf("LEG_1 [%d] angles: coxa %lf femur %lf tibia %lf \n",i,coxa[i],femur[i],tibia[i]);
	}
	publisherSend(coxa,femur,tibia);
}

void controlCallback(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
	ROS_INFO("SIMULATION GUIT START NOW\n");
	if(msg->data[6]==20001)
	{
		if(simulation)
		{
			simulation=false;
			printf("3f");
		}
		else simulation=true;
	}

	GPosX=msg->data[0];
	GPosY=msg->data[1];
	GPosZ=msg->data[2];
	GRotX=msg->data[3];
	GRotY=msg->data[4];
	GRotZ=msg->data[5];
	gait=msg->data[7];
}

void modeCallback(const std_msgs::Int64::ConstPtr& msg)
{
	rotate=msg->data;
}

void bodyCallback(const std_msgs::Int64::ConstPtr& msg)
{
	if(msg->data==1)isBodyMoves?true:false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hexa_kinematics_server");
	ros::NodeHandle n;
	printf("Start!\n");
	ros::Subscriber controlSub=n.subscribe("body_kin",10, controlCallback);
	ros::Subscriber modeSub=n.subscribe("body_kin_mode",10, modeCallback);
	ros::Subscriber bodySub=n.subscribe("body_kin_body",10, bodyCallback);
	jointsSetPub=n.advertise<hexa_msgs::AnglesValues>("/vrep/toSimulator",1000);
	int step=0;
	while(ros::ok())
	{
		if(simulation)
		{
			inverseKinematics(GRotX,GPosY,GPosZ,GRotX,GRotY,GRotZ,step);
			usleep(100000);
			step++;
			printf("ololo");
		}
		else if(0!=GRotX || 0!=GPosY || 0!=GPosZ || 0!=GRotX || 0!=GRotY || 0!=GRotZ)
		{
			inverseKinematics(GRotX,GPosY,GPosZ,GRotX,GRotY,GRotZ,0);
			usleep(100000);
		}
		ros::spinOnce();
	}
	ros::shutdown();
	return 0;
}