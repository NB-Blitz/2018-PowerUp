#ifndef SRC_DRIVE_MANAGER_HPP_
#define SRC_DRIVE_MANAGER_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"
#include "BlitzPIDSource.hpp"


namespace FRC
{
	class Drive_Manager
	{
	public:
		Drive_Manager();

		// Objects
		WPI_TalonSRX Left_Front, Left_Back, Right_Front, Right_Back;
		Solenoid Left_Solenoid, Right_Solenoid;
		AHRS ahrs;
		BlitzPIDSource Left_Front_Source, Left_Back_Source, Right_Front_Source, Right_Back_Source;
		PIDController Left_Front_PID, Left_Back_PID, Right_Front_PID, Right_Back_PID;
		// WPILib PID Method Code

		// Methods
		float getAngle();
		void PIDControlWPIlib(double leftFront, double leftBack, double rightFront, double rightBack);
		void arcadeDrive(double joyY, double joyZ);
		void mecanumDrive(double x, double y, double rotate);
		//double PICorrection(double defaultVal, double encSpeed);
		//double PIDCorrection(double desiredSpeed, double actualSpeed, int motorID);
		void getEncSpeeds();
		void toArcade();
		void toMecanum();
		void testMotorPorts(bool port0, bool port1, bool port2, bool port3);
		void straightDrive(double x, double y, double preAngle);
		//void PIDSetup();
		//double PIDLoop(int motorId, double speed, bool straight);
		//void PIDSetupCTRE();

		// Variables
		double const RATE_FREQUENCY = 2000; // Target Velocity
		double const PROPORTIONAL_GAIN = 1.0; // Proportional multiplier
		double const MAX_HZ = 2600.0; // Max Hz


		//P, I, and D values for motor control
		double PROPORTIONAL_COEFFICIENT = 1;
		double INTEGRAL_COEFFICIENT = 0.1;
		double DERIVATIVE_COEFFICIENT = 0.05;
		double Default_PID[3] = {PROPORTIONAL_COEFFICIENT, INTEGRAL_COEFFICIENT, DERIVATIVE_COEFFICIENT};

		//Arrays holding values in motor control loop
		double baseSpeed[4]; // Base Speeds
		double finalSpeed[4]; // Final Speeds
		double maxMagnitude = 0;

		//Straight Drive variables
		double theta = 0;
		double rotation = 0;

		//Speed holders for motor control
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

		//Integral management variables
		int numberOfLoops = 0;
		int const INTEGRAL_RESET_SECONDS = 1; //Seconds before runningIntegral is reset
		int const INTEGRAL_RESET_LOOPS = INTEGRAL_RESET_SECONDS * (1/0.005); //0.005 seconds per loop

		//Variable for discontinued PI Loop
		double PIOut = 0;

		//Must be true for PID to function with encoders
		bool useEnc = true;


	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
