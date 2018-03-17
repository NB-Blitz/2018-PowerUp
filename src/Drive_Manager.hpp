#ifndef SRC_DRIVE_MANAGER_HPP_
#define SRC_DRIVE_MANAGER_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"

namespace FRC
{
	class Drive_Manager
	{
	public:
		Drive_Manager();

		// Objects
		WPI_TalonSRX Left_Front, Left_Back, Right_Front, Right_Back;
		Compressor Comp;
		Solenoid Left_Solenoid, Right_Solenoid;

		// Methods
		void startCompressor();
		void switchDriveMode(bool isMecanum);
		void mecanumDrive(double joyX, double joyY, double joyZ);
		void arcadeDrive(double joyY, double joyZ);
		void straightDrive(double joyX, double joyY, double joyZ, double angle);
		void fieldControl(double joyX, double joyY, double joyZ, double joyDegrees, double angle);
		double PICorrection(double defaultVal, double encSpeed);
		void getEncSpeeds();
		double getEncSpeed(int motor);
		void testMotorPorts(bool port0, bool port1, bool port2, bool port3);

		// Variables
		double const RATE_FREQUENCY = 2000; // Target Velocity
		double const PROPORTIONAL_GAIN = 1.0; // Proportional multiplier
		double const MAX_HZ = 2600.0; // Max Hz
		double const PI = 3.14159265;

		double baseSpeed[4]; // Base Speeds
		double finalSpeed[4]; // Final Speeds
		double maxMagnitude;

		double theta;
		double rotation;

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
