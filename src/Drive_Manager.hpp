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
		Solenoid Left_Solenoid, Right_Solenoid;
		Encoder Left_Front_Enc;
		PIDController Left_Front_Controller;//, Left_Back_Controller, Right_Front_Controller, Right_Back_Controller;

		// Methods
		void PIDControlWPIlib(double leftFront, double leftBack, double rightFront, double rightBack);
		void arcadeDrive(double joyY, double joyZ);
		void mecanumDrive(double x, double y, double rotate);
		double PICorrection(double defaultVal, double encSpeed);
		double PIDCorrection(double desiredSpeed, double actualSpeed);
		void getEncSpeeds();
		void toArcade();
		void toMecanum();
		void testMotorPorts(bool port0, bool port1, bool port2, bool port3);

		// Variables
		double const RATE_FREQUENCY = 2000; // Target Velocity
		double const PROPORTIONAL_GAIN = 1.0; // Proportional multiplier
		double const MAX_HZ = 2600.0; // Max Hz

		double PROPORTIONAL_COEFFICIENT = 1.0;
		double INTEGRAL_COEFFICIENT = 0.1;
		double DERIVATIVE_COEFFICIENT = 0.1;

		double baseSpeed[4]; // Base Speeds
		double finalSpeed[4]; // Final Speeds
		double maxMagnitude;

		double targetSpeed;
		double currentSpeed;
		double error;
		double propOut;
		double PIOut;
		bool useEnc;

		double integralOut;
		double derivativeOut;
		double PIDOut;

		double encSpeed[4];

		double preError = 0;
		double runningIntegral = 0;
		int numberOfLoops = 0;
		int const INTEGRAL_RESET_SECONDS = 1; //Seconds before runningIntegral is reset
		int const INTEGRAL_RESET_LOOPS = INTEGRAL_RESET_SECONDS * (1/0.005); //0.005 seconds per loop
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
