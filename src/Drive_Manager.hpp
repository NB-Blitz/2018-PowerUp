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
		double PIDCorrection(double defaultVal, double encSpeed, int motorID);
		void getEncSpeeds();
		double getEncSpeed(int motor);
		void testMotorPorts(bool port0, bool port1, bool port2, bool port3);

		// Variables
		double const RATE_FREQUENCY = 2000; // Target Velocity
		double const PROPORTIONAL_GAIN = 1.0; // Proportional multiplier
		double const MAX_HZ = 2600.0; // Max Hz
		double const PI = 3.14159265;

		//P, I, and D values for motor control
		double PROPORTIONAL_COEFFICIENT[4] = {0.75, 0.8, 0.8, 0.75};
		double INTEGRAL_COEFFICIENT[4] = {0.05, -0.065, -0.05, 0.05};
		double DERIVATIVE_COEFFICIENT[4] = {0, 0, 0, 0};

		double baseSpeed[4]; // Base Speeds
		double finalSpeed[4]; // Final Speeds
		double maxMagnitude = 0;
		bool disablePID = false;
		double timeUntilEnablePID = 0;

		//Straight Drive
		double theta;
		double rotation;
		double startAngle;
		bool firstTime;

		//Speed holders for motor control
		double goalSpeed[4];
		double targetSpeed[4];
		double currentSpeed[4];

		//PID Variables/Arrays
		double error[4] = {0,0,0,0};
		double propOut[4] = {0,0,0,0};
		double integralOut[4] = {0,0,0,0};
		double derivativeOut[4] = {0,0,0,0};
		double PIDOut[4] = {0,0,0,0};
		double encSpeed[4] = {0,0,0,0};
		double preError[4] = {0,0,0,0};
		double runningIntegral[4] = {0,0,0,0};
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
