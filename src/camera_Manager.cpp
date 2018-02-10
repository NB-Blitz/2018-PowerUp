#include <WPILib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include "camera_Manager.hpp"

FRC::camera_Manager::camera_Manager():
	pan(0)
	//tilt(1)
{

}

void FRC::camera_Manager::netSetup()
{
	server.sin_family = AF_INET;
	server.sin_port = htons(port);
	server.sin_addr.s_addr = inet_addr("10.51.48.36");

	serverLen = sizeof(server);

	udpSock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
}

void FRC::camera_Manager::grabData()
{
	char xBuffer[256];

	const char* msg = "x";
	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
	recv(udpSock,xBuffer,256,0);
	SmartDashboard::PutString("recieved buffer", xBuffer);
	std::string xPosStr(xBuffer);

	xPos = atoi(xPosStr.c_str());

	char yBuffer[256];

	msg = "y";
	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
	recv(udpSock,yBuffer,256,0);
	SmartDashboard::PutString("recieved buffer", yBuffer);
	std::string yPosStr(yBuffer);

	yPos = atoi(yPosStr.c_str());


	char distBuffer[2];

	msg = "d";
	SmartDashboard::PutNumber("step: ", 1);
	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
	recv(udpSock,distBuffer,256,0);
	//std::string distStr(buffer);
	SmartDashboard::PutString("dist: ", distBuffer);

	dist = atoi(distBuffer);
}

void FRC::camera_Manager::closeNet()
{
	close(udpSock);
}

void FRC::camera_Manager::sendData(std::string data)
{
	const char* msg = data.c_str();
	sendto(udpSock, msg,sizeof(msg),0,(struct sockaddr *)&server,serverLen);
}

void FRC::camera_Manager::trackColor(std::string color)
{
	sendData(color.substr(0,1));
}

void FRC::camera_Manager::camScan()
{
	if(xPos == -1)
	{
		/*if(camPanPos >= 180)
		{
			panDir = -1;
		}
		else if(camPanPos <= 0)
		{
			panDir = 1;
		}

		if(camTiltPos >= 180)
		{
			tiltDir = -1;
		}
		else if(camTiltPos <= 0)
		{
			tiltDir = 1;
		}*/
		angle = 0;
	}
	else
	{
		angle = xPos;
	}

	camPanPos += panDir;
	camTiltPos += tiltDir;

	pan.SetAngle(camPanPos);
	//tilt.SetAngle(camTiltPos);
}
