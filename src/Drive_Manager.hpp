#ifndef SRC_DRIVE_MANAGER_HPP_
#define SRC_DRIVE_MANAGER_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"


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

		// Methods
		float getAngle();
		void arcadeDrive(double joyY, double joyZ);
		void mecanumDrive(double x, double y, double rotate);
		double PIDCorrection(double desiredSpeed, double actualSpeed, int motorID);
		void getEncSpeeds();
		void toArcade();
		void toMecanum();
		void straightDrive(double x, double y, double preAngle);

		// Variables
		double const RATE_FREQUENCY = 2000; // Target Velocity
		double const PROPORTIONAL_GAIN = 1.0; // Proportional multiplier
		double const MAX_HZ = 2600.0; // Max Hz


		//P, I, and D values for motor control
		double PROPORTIONAL_COEFFICIENT[4] = {0.75, 0.8, 0.8, 0.75};
		double INTEGRAL_COEFFICIENT[4] = {0.05, -0.065, -0.05, 0.05};
		double DERIVATIVE_COEFFICIENT[4] = {0, 0, 0, 0};
		double Default_PID[3] = {PROPORTIONAL_COEFFICIENT[1], INTEGRAL_COEFFICIENT[1], DERIVATIVE_COEFFICIENT[1]};

		//Arrays holding values in motor control loop
		double baseSpeed[4]; // Base Speeds
		double finalSpeed[4]; // Final Speeds
		double maxMagnitude = 0;


		double preX = 0;
		double preY = 0;
		double preZ = 0;
		bool disablePID = false;
		double timeUntilEnablePID = 0;

		//Straight Drive variables
		double theta = 0;
		double rotation = 0;
		double preAngle = 0;

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

		//Integral management variables
		int numberOfLoops = 0;
		int const INTEGRAL_RESET_SECONDS = 3; //Seconds before runningIntegral is reset
		int const INTEGRAL_RESET_LOOPS = INTEGRAL_RESET_SECONDS * (1/0.005); //0.005 seconds per loop

		//Variable for discontinued PI Loop
		double PIOut = 0;

		//Must be true for PID to function with encoders
		bool useEnc = true;

		//Number Tester
		double y = 0;

		double propTest = 0;
		double intTest = 0;
		double derivTest = 0;

		//Angle-enhanced PID
		double angle = 0;
		double angleError = 0;
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
