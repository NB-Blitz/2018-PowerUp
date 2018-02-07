#ifndef SRC_DRIVE_MANAGER_HPP_
#define SRC_DRIVE_MANAGER_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "PIDController.h"
#include "Constants.h"

namespace FRC
{
	class Drive_Manager
	{
	public:
		Drive_Manager();

		// Objects
		WPI_TalonSRX Left_Front, Left_Back, Right_Front, Right_Back;
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

		void PIDLoop(double x, double y, double z, bool button);//The PID code that was 'borrowed' from CTRE.

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

		std::string _sd;//??
		int _loops;
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
