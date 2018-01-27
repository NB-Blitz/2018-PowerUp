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
		double PICorrection(double defaultVal, double encSpeed);//activates the PI correction
		void rotate(int degrees);//rotates by a certain angle
		void rotateTo(int degrees);//rotates to a certain value
		void getEncSpeeds();//fills in encSpeed with the encoder values.
		bool leniency(int degree);//Creates leniency, which means it doesn't have to be exact, so it doesn't oscillate.
		double ramp();//A method for moderating acceleration.
		void toggleDrive(bool driveType);//True = tankdrive, False is mecanum.
		void rampStart();//Activates ramping. Takes it to a superlative.
		void rampBack();//Makes it ramp backward.
		void rampForward();//Makes it ramp forward.

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

		double rotateSpeed = 0.5;//The default speed of rotation.

		double timer = 0;//Keeps track of the number of loops in which it ramps up.
		double rampSpeed = 1;//The speed dictated to each motor under Ramp. If it goes in different directions...
		//We will need changes.
		bool rampOn = false;//rampOn basically indicates whether or not
		int rampdirection = 1;//
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
