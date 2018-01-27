#include "WPILib.h"
#include "Drive_Manager.hpp"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"

class Robot: public SampleRobot
{
	FRC::Drive_Manager Drive_Man;
	FRC::Input_Manager Input_Man;
	FRC::Lift_Manager Lift_Man;
	double joyX, joyY, joyZ, joySlide, currentAngle;
	bool toggleDriveButton;

public:
	Robot() :
		Drive_Man(),
		Input_Man(),
		Lift_Man()

	{
		joyX = 0;
		joyY = 0;
		joyZ = 0;
		joySlide = 0;
		currentAngle = 0;
		toggleDriveButton = false;
	}

/*-----------------------------------------------------------------------------------------------
 * 	  _______   _              ____          __  __           _
 *	 |__   __| | |            / __ \        |  \/  |         | |
 *	    | | ___| | ___ ______| |  | |_ __   | \  / | ___   __| | ___
 *	    | |/ _ \ |/ _ \______| |  | | '_ \  | |\/| |/ _ \ / _` |/ _ \
 *	    | |  __/ |  __/      | |__| | |_) | | |  | | (_) | (_| |  __/
 *	    |_|\___|_|\___|       \____/| .__/  |_|  |_|\___/ \__,_|\___|
 *	                                | |
 *	                                |_|
 *----------------------------------------------------------------------------------------------*/
	void OperatorControl()
	{
		Input_Man.resetNav();

		while (IsOperatorControl() && IsEnabled())
		{
			// VARIABLE SETTING
			Drive_Man.getEncSpeeds();
			joyX = Input_Man.getX();
			joyY = Input_Man.getY();
			joyZ = Input_Man.getZ();
			joySlide = Input_Man.getSlide();
			currentAngle = Input_Man.getAngle();
			toggleDriveButton = Input_Man.getDriveButton();

			Drive_Man.toggleDrive();
			// DRIVE
			if (toggleDriveButton)
			{
				// Pneumatic cylinders out
				Drive_Man.arcadeDrive(joyY, joyZ);
			}
			else
			{
				// Pneumatic cylinders in
				Drive_Man.mecanumDrive(joyX, joyY, joyZ * .7);
			}

			// LIFT
			//Lift_Man.moveLift(joyY);
			//Lift_Man.moveLiftTo(joySlide);

			Wait(0.005);
		}
	}

/*-----------------------------------------------------------------------------------------------
 *	  _______        _     __  __           _
 *	 |__   __|      | |   |  \/  |         | |
 *	    | | ___  ___| |_  | \  / | ___   __| | ___
 *	    | |/ _ \/ __| __| | |\/| |/ _ \ / _` |/ _ \
 *	    | |  __/\__ \ |_  | |  | | (_) | (_| |  __/
 *	    |_|\___||___/\__| |_|  |_|\___/ \__,_|\___|
 *
 *----------------------------------------------------------------------------------------------*/
	void Test()
	{
		while (IsTest() && IsEnabled())
		{
			Lift_Man.resetLift();
		}
	}
};

START_ROBOT_CLASS(Robot)
