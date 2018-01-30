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
	bool isArcade;

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
		isArcade = false;
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
			joyX = Input_Man.getAxis(0);
			joyY = Input_Man.getAxis(1);
			joyZ = Input_Man.getAxis(2);
			joySlide = Input_Man.getAxis(3);
			currentAngle = Input_Man.getAngle();
			isArcade = Input_Man.getJoyButton(1);

			// DRIVE
			if (isArcade)
			{
				Drive_Man.solenoidsOut();
				Drive_Man.arcadeDrive(joyY, joyZ);
			}
			else
			{
				Drive_Man.solenoidsIn();
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
			//Lift_Man.resetLift();
		}
	}
};

START_ROBOT_CLASS(Robot)
