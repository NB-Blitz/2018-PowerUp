#include "WPILib.h"
#include "Camera_Manager.hpp"
#include "iostream"

FRC::Camera_Manager::Camera_Manager()
{
	Tilt = new Servo(1);
	Pan = new Servo(0);
}

void FRC::Camera_Manager::camSetup()
{
	Tilt = new Servo(1);
	Pan = new Servo(0);

	server.sin_family = AF_INET;
	server.sin_port = htons(port);
	server.sin_addr.s_addr = inet_addr(CAMERA_IP.c_str());

	serverLen = sizeof(server);

	udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	setup = true;
}

void FRC::Camera_Manager::grabData()
{
	char xBuffer[256];

	const char* msg = "x";
	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
	recv(udpSock,xBuffer,256,0);

	//SmartDashboard::PutString("recieved buffer", xBuffer);
	std::string xPosStr(xBuffer);

	xPos = atoi(xPosStr.c_str());

	char yBuffer[256];

	msg = "y";
	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
	recv(udpSock,yBuffer,256,0);

	SmartDashboard::PutString("recieved buffer", yBuffer);
	std::string yPosStr(yBuffer);

	yPos = atoi(yPosStr.c_str());

//	char distBuffer[2];

//	msg = "d";
//	SmartDashboard::PutNumber("step: ", 1);
//	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
//	recv(udpSock,distBuffer,256,0);

//	//std::string distStr(buffer);
//	SmartDashboard::PutString("dist: ", distBuffer);

//	dist = atoi(distBuffer);
}

void FRC::Camera_Manager::closeNet()
{
	close(udpSock);
}

void FRC::Camera_Manager::sendData(std::string data)
{
	const char* msg = data.c_str();
	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
}

void FRC::Camera_Manager::trackColor(std::string color)
{
	sendData(color.substr(0,1));
}

void FRC::Camera_Manager::camScan(int autoGoal)
{
	if(xPos > 90 || xPos < -90)
	{
		targetFound = false;
		if(camPanPos >= MAX_PAN)
		{
			panDir = -1;
			camPanPos = MAX_PAN;
		}
		else if(camPanPos <= MIN_PAN)
		{
			panDir = 1;
			camPanPos = MIN_PAN;
		}

		angle = 0;
		dist = -1;

		camPanPos += panDir;
	}
	else
	{
		targetFound = true;
		if(xPos < 0)
		{
			camPanPos -= .25;
		}
		else if(xPos > 0)
		{
			camPanPos += .25;
		}

		angle = camPanPos - 90;

		if(yPos > 1 && camTiltPos < MIN_TILT)
		{
			camTiltPos -= .25;
		}
		else if(yPos < -1 && camTiltPos < MAX_TILT)
		{
			camTiltPos += .25;
		}

//		dist = tan(camTiltPos) * (M_PI/180) * CAM_HEIGHT;

		if(camPanPos >= MAX_PAN)
		{
			camPanPos = MAX_PAN;
		}
		else if(camPanPos <= MIN_PAN)
		{
			camPanPos = MIN_PAN;
		}
	}

	setPanPos(camPanPos);
}

void FRC::Camera_Manager::setPanPos(double pos)
{
	Pan->Set(pos/180);
}

void FRC::Camera_Manager::setTiltPos(double pos)
{
	Tilt->Set(pos/180);
}
