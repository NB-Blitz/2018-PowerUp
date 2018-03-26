#ifndef SRC_CAMERA_MANAGER_HPP_
#define SRC_CAMERA_MANAGER_HPP_

#include "WPILib.h"
#include "arpa/inet.h"
#include "unistd.h"
#include "iostream"

namespace FRC
{
	class Camera_Manager
	{
	public:
		Camera_Manager();

		Servo *Tilt;
		Servo *Pan;

		sockaddr_in server;
		unsigned short port = 5800;
		int udpSock = -1;
		socklen_t serverLen = 0;

		int xPos = -1;
		int yPos = -1;
		int dist = -1;
		int angle = -1;

		double camPanPos = 90;
		double camTiltPos = 90;
		double panDir = 1;
		int tiltDir = 1;
		int defaultTilt = 90;
		bool targetFound = false;
		bool setup = false;

		void camSetup();
		void grabData();
		void trackColor(std::string color);
		void sendData(std::string data);
		void closeNet();

		void camScan(int autoGoal);
		void setPanPos(double pos);
		void setTiltPos(double pos);

		const std::string CAMERA_IP = "10.51.48.36";

		const int CAM_HEIGHT = 50;
		const int MIN_TILT = 45;
		const int MAX_TILT = 180;
		const int MIN_PAN = 45;
		const int MAX_PAN = 135;

		const int DEFAULT_SWITCH_TILT = 130;
		const int DEFAULT_SCALE_TILT = 90;

		const int RIGHT_SWITCH_PAN = 128;
		const int LEFT_SWITCH_PAN =52;
		const int CENTER_SWITCH_LEFT_PAN = 132;
		const int CENTER_SWITCH_RIGHT_PAN = 68;

		const int RIGHT_SCALE_PAN = 129;
		const int LEFT_SCALE_PAN = 67;
		const int CENTER_SCALE_LEFT_PAN = 107;
		const int CENTER_SCALE_RIGHT_PAN = 72;
	};
}

#endif /* SRC_CAMERAMANAGER_HPP_ */
