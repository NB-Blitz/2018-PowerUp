#ifndef SRC_DRIVE_MANAGER_HPP_
#define SRC_DRIVE_MANAGER_HPP_

#include "WPILib.h"
#include "Input_Manager.hpp"
#include "ctre/Phoenix.h"

namespace FRC
{
	class Drive_Manager
	{
	public:
		Drive_Manager();

		// Objects
		FRC::Input_Manager Input_Man;
		WPI_TalonSRX Left_Back, Left_Front, Right_Back, Right_Front;
		Solenoid Left_Solenoid, Right_Solenoid;

		// Methods
		void arcadeDrive(double joyY, double joyZ);
		void mecanumDrive(double x, double y, double rotate);
		double PICorrection(double defaultVal, double encSpeed);
		void rotate(int degrees);
		void rotateTo(int degrees);
		void getEncSpeeds();
		void solenoidsOut();
		void solenoidsIn();
		void testMotorPorts(bool port0, bool port1, bool port2, bool port3);

		// Variables
		double const RATE_FREQUENCY = 2000; // Target Velocity
		double const PROPORTIONAL_GAIN = 1.0; // Proportional multiplier
		double const MAX_HZ = 2600.0; // Max Hz

		double baseSpeed[4]; // Base Speeds
		double finalSpeed[4]; // Final Speeds
		double maxMagnitude;

		double targetSpeed;
		double currentSpeed;
		double error;
		double propOut;
		double PIOut;
		bool useEnc;

		double encSpeed[4];
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
