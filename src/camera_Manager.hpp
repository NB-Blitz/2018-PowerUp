#ifndef SRC_CAMERA_MANAGER_HPP_
#define SRC_CAMERA_MANAGER_HPP_

#include <WPILib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

namespace FRC
{
	class camera_Manager
	{
	public:
		camera_Manager();

		Servo pan;
		//Servo tilt;

		sockaddr_in server;
		unsigned short port = 5800;
		int udpSock = -1;
		socklen_t serverLen = 0;

		int xPos = -1;
		int yPos = -1;
		int dist = -1;
		int angle = -1;

		int camPanPos = 0;
		int camTiltPos = 0;
		int panDir = 1;
		int tiltDir = 1;

		void netSetup();
		void grabData();
		void trackColor(std::string color);
		void sendData(std::string data);
		void closeNet();

		void camScan();

	private:

	};
}



#endif /* SRC_CAMERAMANAGER_HPP_ */
