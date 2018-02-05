#ifndef SRC_INPUT_MANAGER_HPP_
#define SRC_INPUT_MANAGER_HPP_

#include "WPILib.h"
#include "AHRS.h"

namespace FRC
{
	class Input_Manager
	{
	public:
		Input_Manager();

		// Object Declarations
		AHRS Nav;
		Joystick Stick, Controller;

		// Methods
		double xRamp(double joyX);
		double yRamp(double joyY);
		double zRamp(double joyZ);

		double * ramp(double joyX, double joyY, double joyZ);
		double prevXRamp(double joyX);
		double prevYRamp(double joyY);
		double prevZRamp(double joyZ);

		double getAxis(int axis);
		bool getJoyButton(int button);
		double getControllerAxis(int axis);
		bool getControllerButton(int button);

		double getAngle();
		void resetNav();

		// Variables
		const double ACCEL_CAP = .05; // The maximum acceleration. It won't let you go much faster than this... THough this code barely makes sense.
		const double DEADBAND = .2;
		const double RAMP_RATE = 1/(.5 * 200.0); //.5 = time in seconds, 200 = scan rate

		double sendspeedX; // The speed that gets sent.
		double sendspeedY;
		double sendspeedZ;
		double goalspeedX; // What speed you're trying to get to.
		double goalspeedY;
		double goalspeedZ;
		int roundsX; // Some delaying stuff, lets me increment the acceleration.
		int roundsY;
		int roundsZ;

		double joyXRaw;
		double joyYRaw;
		double joyZRaw;
		double finalAxis[3];
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
